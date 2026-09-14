#!/usr/bin/env python3
"""Shared spawn/delete-by-name helper for Gazebo world/environment models.

Extracted out of sim_bridge_node.py and sim_connector_bridge_gazebo.py, which
had each independently grown their own copy of the same "read a model.sdf
once, then spawn/delete it live by name via /gazebo/spawn_sdf_model and
/gazebo/delete_model" pattern for the obstacle_course model. Generalizes that
pattern to any model directory under sim_container/models/ that has a
model.sdf in it -- including models scan_to_environment.py generates from a
phone scan -- so a new environment doesn't need a third hand-copied version
of this logic (see docs/SCAN_TO_SIM_ENVIRONMENT_PLAN.md section 7).

Only one environment model is meant to be live in the world at a time --
EnvironmentModelSpawner enforces that by deleting whatever was previously
spawned before spawning a newly-requested one, rather than letting two
environment models' geometry overlap.
"""
import os

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel, DeleteModel

SPAWN_MODEL_SERVICE = "/gazebo/spawn_sdf_model"
DELETE_MODEL_SERVICE = "/gazebo/delete_model"
GAZEBO_SERVICE_WAIT_SEC = 5.0

MODELS_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models")


ENVIRONMENT_OPTION_MARKER = ".environment_option"


def list_environment_models():
    """Directory-scans sim_container/models/ for spawnable ENVIRONMENT
    models -- an immediate subfolder counts only if it has both a model.sdf
    AND an ENVIRONMENT_OPTION_MARKER file. The marker is opt-in on purpose:
    sim_container/models/ also holds non-environment models (generic_rover,
    camera_rig, ...) and, per a naming-collision heads-up from a concurrent
    session's dimensions.yaml/generate_model_sdf.py work, may in future hold
    per-model dimension-variant folders that also have a model.sdf but are
    NOT meant to be offered as a selectable "environment" -- requiring an
    explicit marker avoids scooping those up by accident. scan_to_environment.py
    writes the marker for every model it generates; obstacle_course carries
    it too (added alongside this function) since it already was a real
    environment option before this dynamic-listing existed.
    """
    if not os.path.isdir(MODELS_ROOT):
        return []
    names = []
    for entry in sorted(os.listdir(MODELS_ROOT)):
        model_dir = os.path.join(MODELS_ROOT, entry)
        if (os.path.isfile(os.path.join(model_dir, "model.sdf")) and
                os.path.isfile(os.path.join(model_dir, ENVIRONMENT_OPTION_MARKER))):
            names.append(entry)
    return names


class EnvironmentModelSpawner:
    """Tracks which named environment model (if any) is currently spawned in
    the live Gazebo world, and spawns/deletes by name on request. One
    instance is shared by a bridge for however many named models it wants to
    offer -- today: obstacle_course, plus whatever scan_to_environment.py has
    produced.
    """

    def __init__(self, log_prefix):
        self.log_prefix = log_prefix
        self.spawned_name = None
        self._sdf_cache = {}

    def is_spawned(self, name):
        return self.spawned_name == name

    def set_active_model(self, model_name):
        """Spawns `model_name`, first deleting whatever environment model was
        previously spawned (only one is live at a time, so two environments'
        geometry never overlaps). `model_name=None` means "nothing spawned"
        (e.g. the FLAT_GROUND option) -- just deletes whatever was active.
        """
        if model_name == self.spawned_name:
            return True
        if self.spawned_name is not None:
            self._delete(self.spawned_name)
        if model_name is None:
            return True
        return self._spawn(model_name)

    def _load_sdf(self, name):
        if name not in self._sdf_cache:
            path = os.path.join(MODELS_ROOT, name, "model.sdf")
            try:
                with open(path, "r") as f:
                    self._sdf_cache[name] = f.read()
            except Exception as e:
                rospy.logwarn("%s: could not read %s: %s", self.log_prefix, path, str(e))
                self._sdf_cache[name] = None
        return self._sdf_cache[name]

    def _spawn(self, name):
        sdf = self._load_sdf(name)
        if sdf is None:
            return False
        try:
            rospy.wait_for_service(SPAWN_MODEL_SERVICE, timeout=GAZEBO_SERVICE_WAIT_SEC)
            spawn = rospy.ServiceProxy(SPAWN_MODEL_SERVICE, SpawnModel)
            resp = spawn(name, sdf, "", Pose(), "world")
            if resp.success:
                self.spawned_name = name
                rospy.loginfo("%s: spawned environment model '%s'", self.log_prefix, name)
                return True
            rospy.logwarn("%s: spawn of '%s' failed: %s",
                           self.log_prefix, name, resp.status_message)
            return False
        except Exception as e:
            rospy.logwarn("%s: spawn of '%s' failed: %s", self.log_prefix, name, str(e))
            return False

    def _delete(self, name):
        try:
            rospy.wait_for_service(DELETE_MODEL_SERVICE, timeout=GAZEBO_SERVICE_WAIT_SEC)
            delete = rospy.ServiceProxy(DELETE_MODEL_SERVICE, DeleteModel)
            resp = delete(name)
            if resp.success:
                if self.spawned_name == name:
                    self.spawned_name = None
                rospy.loginfo("%s: deleted environment model '%s'", self.log_prefix, name)
                return True
            rospy.logwarn("%s: delete of '%s' failed: %s",
                           self.log_prefix, name, resp.status_message)
            return False
        except Exception as e:
            rospy.logwarn("%s: delete of '%s' failed: %s", self.log_prefix, name, str(e))
            return False
