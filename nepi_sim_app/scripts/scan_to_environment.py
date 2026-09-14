#!/usr/bin/env python3
"""Converts a Stray Scanner phone-scan export (LiDAR depth + IMU + RGB video +
ARKit pose -- see docs/SCAN_TO_SIM_ENVIRONMENT_PLAN.md) into a spawnable Gazebo
model: a decimated visual mesh plus a convex-decomposed collision mesh, wrapped
in a model.sdf/model.config pair matching sim_container/models/obstacle_course/'s
structure so it can be spawned/deleted by name the same way.

Pipeline (see the plan doc for citations): confidence-filtered depth -> Open3D
TSDF fusion -> marching-cubes mesh (the kekeblom/StrayVisualizer reference
approach), extended with quadric decimation (visual mesh) and CoACD convex
decomposition (collision mesh, since Gazebo's physics engines are unreliable
against the raw concave/non-manifold mesh TSDF fusion produces).

Standalone script -- no rospy/ROS import, meant to run offline on the dev VM
(TSDF fusion + convex decomposition over ~1400 frames is real compute, not
something to run inline in an upload request handler).

Usage:
    python3 scan_to_environment.py <scan_dir> <scan_name> [options]

Example:
    python3 scan_to_environment.py \\
        sim_container/scan_data/raw/office_779206be34 office_scan
"""
import argparse
import csv
import os
import sys

import cv2
import numpy as np
import open3d as o3d


DEPTH_MM_TO_M = 1.0 / 1000.0

# ARKit's camera frame (X-right, Y-up, Z-backward -- camera looks down -Z, the
# same convention as OpenGL) needs a fixed axis flip to become the CV/Open3D
# camera frame (X-right, Y-down, Z-forward -- camera looks down +Z) that
# PinholeCameraIntrinsic/TSDF integration assume. This is the standard
# ARKit -> COLMAP/Open3D conversion (also used by Record3D-style converters).
_ARKIT_CAMERA_TO_CV_CAMERA = np.diag([1.0, -1.0, -1.0, 1.0])

# ARKit's world frame is Y-up (X-right, Z-toward-viewer). Gazebo/ROS worlds are
# Z-up. This fixed +90-degree-about-X remap (X, Y, Z)_arkit -> (X, -Z, Y)_gazebo
# is a proper rotation (determinant +1, right-handedness preserved) that turns
# "up" into Z without introducing a reflection.
_ARKIT_WORLD_TO_GAZEBO_WORLD = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
    [0.0, 1.0, 0.0],
])


def _quat_xyzw_to_matrix(q_xyzw):
    x, y, z, w = q_xyzw
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def load_odometry(scan_dir, depth_width, depth_height, rgb_width, rgb_height):
    """Parses odometry.csv into one dict per frame: frame index, the 4x4
    world<-camera extrinsic (CV convention, ready for TSDFVolume.integrate),
    and pinhole intrinsics scaled from the RGB-resolution values odometry.csv
    stores down to the depth map's native resolution -- confirmed by
    inspection that fx/fy/cx/cy in this file are at the RGB video's
    resolution (cx/cy sit at roughly half the RGB width/height), the same
    thing the reference StrayVisualizer implementation scales down.
    """
    sx = depth_width / float(rgb_width)
    sy = depth_height / float(rgb_height)
    frames = []
    with open(os.path.join(scan_dir, "odometry.csv"), "r") as f:
        # Stray Scanner's CSV header/values use ", " (space after the comma)
        # -- DictReader takes the header literally, so "frame" would
        # otherwise miss " frame". Strip whitespace off both sides.
        reader = csv.DictReader(f)
        reader.fieldnames = [name.strip() for name in reader.fieldnames]
        for raw_row in reader:
            row = {k: v.strip() if isinstance(v, str) else v for k, v in raw_row.items()}
            frame_idx = int(row["frame"])
            pos = np.array([float(row["x"]), float(row["y"]), float(row["z"])])
            quat_xyzw = np.array([float(row["qx"]), float(row["qy"]),
                                   float(row["qz"]), float(row["qw"])])
            T_WC_arkit = np.eye(4)
            T_WC_arkit[:3, :3] = _quat_xyzw_to_matrix(quat_xyzw)
            T_WC_arkit[:3, 3] = pos
            T_WC_cv = T_WC_arkit @ _ARKIT_CAMERA_TO_CV_CAMERA
            extrinsic = np.linalg.inv(T_WC_cv)  # world -> camera, CV convention
            frames.append({
                "frame_idx": frame_idx,
                "extrinsic": extrinsic,
                "intrinsic": o3d.camera.PinholeCameraIntrinsic(
                    depth_width, depth_height,
                    float(row["fx"]) * sx, float(row["fy"]) * sy,
                    float(row["cx"]) * sx, float(row["cy"]) * sy),
            })
    return frames


def load_filtered_depth_m(scan_dir, frame_idx, min_confidence):
    """16-bit depth PNG (millimeters) -> float32 meters, zeroing any pixel
    whose confidence.png value is below min_confidence (0/1/2, ARKit's
    ARConfidenceLevel) -- the same confidence-gating step the reference
    StrayVisualizer implementation applies before fusion.
    """
    name = "%06d.png" % frame_idx
    depth_mm = cv2.imread(os.path.join(scan_dir, "depth", name), cv2.IMREAD_UNCHANGED)
    if depth_mm is None:
        return None
    depth_m = depth_mm.astype(np.float32) * DEPTH_MM_TO_M
    confidence = cv2.imread(os.path.join(scan_dir, "confidence", name), cv2.IMREAD_UNCHANGED)
    if confidence is not None:
        depth_m[confidence < min_confidence] = 0.0
    return depth_m


def fuse_tsdf(scan_dir, voxel_length, sdf_trunc, depth_trunc, min_confidence, frame_stride):
    sample_depth = cv2.imread(
        os.path.join(scan_dir, "depth", "000000.png"), cv2.IMREAD_UNCHANGED)
    depth_h, depth_w = sample_depth.shape[:2]

    video = cv2.VideoCapture(os.path.join(scan_dir, "rgb.mp4"))
    rgb_w = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    rgb_h = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
    video.release()

    frames = load_odometry(scan_dir, depth_w, depth_h, rgb_w, rgb_h)
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=voxel_length,
        sdf_trunc=sdf_trunc,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor,
    )
    # integrate() requires an RGBDImage even in NoColor mode -- a blank color
    # plane satisfies the API without implying any texture/color pass.
    blank_color = o3d.geometry.Image(np.zeros((depth_h, depth_w, 3), dtype=np.uint8))

    used = 0
    for frame in frames[::frame_stride]:
        depth_m = load_filtered_depth_m(scan_dir, frame["frame_idx"], min_confidence)
        if depth_m is None or not np.any(depth_m > 0):
            continue
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            blank_color, o3d.geometry.Image(depth_m),
            depth_scale=1.0, depth_trunc=depth_trunc, convert_rgb_to_intensity=False)
        volume.integrate(rgbd, frame["intrinsic"], frame["extrinsic"])
        used += 1
    print("scan_to_environment: fused %d/%d frames (stride=%d)" %
          (used, len(frames), frame_stride))
    if used == 0:
        raise RuntimeError("No frames had any usable depth -- check min_confidence")

    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    return mesh


def to_gazebo_frame_and_recenter(mesh):
    """Remaps ARKit's Y-up world into Gazebo's Z-up world, then recenters the
    result so the model spawns with its footprint centered at the world
    origin and its lowest point sitting on the ground plane (z=0) -- ready to
    drop directly into generic_rover.world without manual placement.
    """
    verts = np.asarray(mesh.vertices) @ _ARKIT_WORLD_TO_GAZEBO_WORLD.T
    center_xy = (verts[:, :2].min(axis=0) + verts[:, :2].max(axis=0)) / 2.0
    verts[:, 0] -= center_xy[0]
    verts[:, 1] -= center_xy[1]
    verts[:, 2] -= verts[:, 2].min()
    mesh.vertices = o3d.utility.Vector3dVector(verts)
    return mesh


def clean_mesh(mesh, min_cluster_fraction=0.02):
    """Drops small disconnected components (isolated noise blobs TSDF fusion
    tends to leave behind at scan-volume edges), keeping only clusters at
    least min_cluster_fraction of the largest cluster's triangle count.
    """
    triangle_clusters, cluster_n_triangles, _ = mesh.cluster_connected_triangles()
    cluster_n_triangles = np.asarray(cluster_n_triangles)
    triangle_clusters = np.asarray(triangle_clusters)
    if len(cluster_n_triangles) == 0:
        return mesh
    keep_threshold = cluster_n_triangles.max() * min_cluster_fraction
    triangles_to_remove = cluster_n_triangles[triangle_clusters] < keep_threshold
    mesh.remove_triangles_by_mask(triangles_to_remove)
    mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    return mesh


def decimate_mesh(mesh, target_triangles):
    """Quadric decimation to a fixed triangle budget. Used twice, with two
    different budgets: once (finer) for the visual mesh, once (much coarser)
    to pre-simplify the mesh CoACD runs on -- collision fidelity doesn't need
    anywhere near the visual mesh's detail, and running CoACD directly on a
    million-triangle mesh is both slow and produces far more hulls than a
    Gazebo collision model should carry.
    """
    if len(mesh.triangles) <= target_triangles:
        return mesh
    decimated = mesh.simplify_quadric_decimation(target_number_of_triangles=target_triangles)
    decimated.compute_vertex_normals()
    return decimated


def convex_decompose_collision(mesh, threshold, max_convex_hull):
    """Runs CoACD on the (already cleaned/decimated) mesh, returning a list of
    (vertices, faces) convex hulls -- Gazebo-collision-safe, unlike the raw
    concave TSDF mesh. See docs/SCAN_TO_SIM_ENVIRONMENT_PLAN.md section 4 for
    why this step is required rather than using the visual mesh directly as
    collision geometry.
    """
    import coacd
    coacd_mesh = coacd.Mesh(np.asarray(mesh.vertices), np.asarray(mesh.triangles))
    parts = coacd.run_coacd(coacd_mesh, threshold=threshold, max_convex_hull=max_convex_hull)
    print("scan_to_environment: convex-decomposed collision into %d hulls" % len(parts))
    return parts


def save_topdown_snapshot(mesh, hulls, out_path):
    """Renders a top-down (Gazebo X/Y) scatter of the visual mesh vertices
    plus collision-hull outlines to a PNG -- a quick sanity check that the
    reconstruction looks like a room/course layout rather than noise, since
    this script has no 3D viewer available in its running environment.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    verts = np.asarray(mesh.vertices)
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.scatter(verts[:, 0], verts[:, 1], s=0.2, c=verts[:, 2], cmap="viridis")
    for hull_verts, hull_faces in hulls:
        hv = np.asarray(hull_verts)
        ax.plot(np.append(hv[:, 0], hv[0, 0]), np.append(hv[:, 1], hv[0, 1]),
                linewidth=0.3, color="red", alpha=0.4)
    ax.set_aspect("equal")
    ax.set_xlabel("Gazebo X (m)")
    ax.set_ylabel("Gazebo Y (m)")
    ax.set_title("Top-down sanity check: %d verts, %d hulls" % (len(verts), len(hulls)))
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print("scan_to_environment: wrote top-down sanity snapshot to %s" % out_path)


MODEL_SDF_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{scan_name}">
    <!-- Auto-generated by sim_container/scripts/scan_to_environment.py from a
         phone (Stray Scanner) scan -- see docs/SCAN_TO_SIM_ENVIRONMENT_PLAN.md.
         Static: pure world geometry, no dynamics needed. Visual and collision
         geometry are deliberately different meshes (Gazebo's own documented
         best practice): the visual mesh is the full decimated reconstruction;
         collision geometry is {n_hulls} convex hulls (CoACD decomposition),
         since Gazebo's physics engines are unreliable against the raw
         concave/non-manifold mesh a real scan reconstructs to. -->
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <mesh><uri>model://{scan_name}/meshes/visual.obj</uri></mesh>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Grey</name>
          </script>
        </material>
      </visual>
{collisions}
    </link>
  </model>
</sdf>
"""

COLLISION_TEMPLATE = """      <collision name="collision_{i}">
        <geometry>
          <mesh><uri>model://{scan_name}/meshes/collision_{i}.obj</uri></mesh>
        </geometry>
      </collision>"""

MODEL_CONFIG_TEMPLATE = """<?xml version="1.0"?>

<model>
  <name>{display_name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>

  <author>
    <name>Numurus</name>
    <email>nepi@numurus.com</email>
  </author>

  <description>
    Auto-generated from a phone LiDAR scan ({scan_name}) by
    scan_to_environment.py. Reconstructed via Open3D TSDF fusion, decimated
    for the visual mesh, and convex-decomposed (CoACD, {n_hulls} hulls) for
    collision geometry. Spawned/deleted by name the same way as
    obstacle_course (see docs/SCAN_TO_SIM_ENVIRONMENT_PLAN.md). Accuracy
    caveats (LiDAR range/noise, VIO drift over the scan) apply -- see the
    plan doc before treating this as survey-grade geometry.
  </description>
</model>
"""


def write_model(scan_name, visual_mesh, hulls, models_root):
    import trimesh as _trimesh  # local import: only needed for OBJ export of hulls

    out_dir = os.path.join(models_root, scan_name)
    meshes_dir = os.path.join(out_dir, "meshes")
    os.makedirs(meshes_dir, exist_ok=True)

    o3d.io.write_triangle_mesh(os.path.join(meshes_dir, "visual.obj"), visual_mesh)

    collisions_xml = []
    for i, (hull_verts, hull_faces) in enumerate(hulls):
        hull_mesh = _trimesh.Trimesh(vertices=hull_verts, faces=hull_faces, process=False)
        hull_mesh.export(os.path.join(meshes_dir, "collision_%d.obj" % i))
        collisions_xml.append(COLLISION_TEMPLATE.format(scan_name=scan_name, i=i))

    with open(os.path.join(out_dir, "model.sdf"), "w") as f:
        f.write(MODEL_SDF_TEMPLATE.format(
            scan_name=scan_name, n_hulls=len(hulls),
            collisions="\n".join(collisions_xml)))

    # Opts this model into environment_models.py's dynamic environment-options
    # list -- see that file's ENVIRONMENT_OPTION_MARKER docstring for why an
    # explicit marker is required rather than any model.sdf-having folder
    # qualifying.
    open(os.path.join(out_dir, ".environment_option"), "w").close()

    with open(os.path.join(out_dir, "model.config"), "w") as f:
        f.write(MODEL_CONFIG_TEMPLATE.format(
            display_name=scan_name.replace("_", " ").title(),
            scan_name=scan_name, n_hulls=len(hulls)))

    print("scan_to_environment: wrote model to %s" % out_dir)
    return out_dir


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("scan_dir", help="Path to a raw Stray Scanner session folder "
                         "(contains odometry.csv, depth/, confidence/, rgb.mp4)")
    parser.add_argument("scan_name", help="Output model name (becomes the Gazebo model "
                         "name and the folder under sim_container/models/)")
    parser.add_argument("--models-root", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "models"))
    parser.add_argument("--processed-root", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "scan_data", "processed"))
    parser.add_argument("--voxel-length", type=float, default=0.02,
                         help="TSDF voxel size in meters (default 0.02)")
    parser.add_argument("--sdf-trunc", type=float, default=0.06,
                         help="TSDF truncation distance in meters (default 0.06)")
    parser.add_argument("--depth-trunc", type=float, default=4.0,
                         help="Ignore depth beyond this range in meters (default 4.0, "
                         "per the LiDAR accuracy caveats in the plan doc)")
    parser.add_argument("--min-confidence", type=int, default=1, choices=[0, 1, 2],
                         help="Minimum ARKit confidence (0/1/2) to keep a depth pixel "
                         "(default 1)")
    parser.add_argument("--frame-stride", type=int, default=1,
                         help="Use every Nth frame (default 1 = all frames)")
    parser.add_argument("--visual-triangles", type=int, default=40000,
                         help="Target triangle count for the decimated visual mesh")
    parser.add_argument("--collision-pre-decimate-triangles", type=int, default=6000,
                         help="Simplify to this many triangles before running convex "
                         "decomposition -- keeps hull count and CoACD runtime sane")
    parser.add_argument("--collision-threshold", type=float, default=0.1,
                         help="CoACD concavity threshold -- higher = fewer/coarser hulls")
    parser.add_argument("--max-convex-hulls", type=int, default=48,
                         help="CoACD max hull count, -1 = automatic/unbounded")
    parser.add_argument("--no-topdown-snapshot", action="store_true",
                         help="Skip writing the top-down sanity-check PNG")
    args = parser.parse_args()

    mesh = fuse_tsdf(args.scan_dir, args.voxel_length, args.sdf_trunc,
                      args.depth_trunc, args.min_confidence, args.frame_stride)
    mesh = to_gazebo_frame_and_recenter(mesh)
    mesh = clean_mesh(mesh)
    print("scan_to_environment: cleaned mesh has %d vertices / %d triangles" %
          (len(mesh.vertices), len(mesh.triangles)))

    visual_mesh = decimate_mesh(mesh, args.visual_triangles)
    collision_input = decimate_mesh(mesh, args.collision_pre_decimate_triangles)
    hulls = convex_decompose_collision(collision_input, args.collision_threshold,
                                        args.max_convex_hulls)

    if not args.no_topdown_snapshot:
        os.makedirs(args.processed_root, exist_ok=True)
        save_topdown_snapshot(
            visual_mesh, hulls,
            os.path.join(args.processed_root, "%s_topdown.png" % args.scan_name))

    write_model(args.scan_name, visual_mesh, hulls, args.models_root)


if __name__ == "__main__":
    sys.exit(main())
