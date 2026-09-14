# Depth map reference examples

Reference data showing the target look for the sim cameras' depth-map output:
close = blue, far = down the rainbow toward red/yellow (jet colormap).

Each capture is a matched triplet, same timestamp prefix:
- `*-color_image.png` — the plain color frame
- `*-depth_map.npy` — raw depth array (float32, millimeters, 10000.0 = no valid
  return / out of range)
- `*-depth_map_image.png` — the pre-rendered colorized depth image this look
  is modeled after

Source: TUM RGB-D SLAM dataset (`freiburg2_pioneer_360`), used here purely as
a visual reference for the sim's own depth-map colorization, not replayed
into the sim directly.
