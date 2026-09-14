#!/usr/bin/env python3
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_apps) repo
# (see https://https://github.com/nepi-engine/nepi_apps)
#
# License: nepi applications are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#

# Runs on a laptop/VM registered as a "shared_storage" Sim Connector OS
# instance (see os_instance_registry.py's connection_mode -- additive
# alongside the existing SSH/reverse-tunnel instance type, not a replacement
# for it). Requested live (2026-09-04): reverse SSH from the device into an
# operator's own machine is a real security concern (a trusted key that can
# run ARBITRARY remote commands); nepi_storage is already a shared SMB drive
# both sides can read/write, so use that as the transport instead -- no
# listening port anywhere, no SSH key trusted for arbitrary command
# execution. The trust boundary becomes "whatever this script is willing to
# do", which is exactly: run the same launch_command/stop_command/
# install_command/check_installed_command text simulator_launch_targets.yaml
# already authors for the SSH path, just locally instead of over SSH --
# nothing this script does is a capability the operator's own machine didn't
# already have.
#
# Protocol (files under <storage_root>/databases/nepi_app_sim_connector/
# vm_commands/<instance_id>/, i.e. the SAME nepi_storage tree
# os_instance_registry.py's own OS_INSTANCES_STORAGE_DIR already lives
# under -- see that module's docstring for why instance data belongs in the
# read/write database tree, not the checked-in config tree). Two distinct
# file shapes, for two different needs:
#
#   1. deploy_state.yaml -- ONE persistent file, the real deploy/kill path
#      (requested live 2026-09-04: "all that the user will have is ssh from
#      a linux os to the nepi device [never the other direction]...
#      installation will already be dealt with in a setup script. the
#      deploy and kill commands... need to be dealt with 0s and 1s in nepi
#      storage" -- a real deployment has NO device-to-VM network path at
#      all, reverse-tunnel included, so this is not a fallback transport,
#      it is THE transport). Device writes:
#        control: {desired_target: <target_key, '' to stop whatever is
#                   running>, last_updated: <epoch seconds>}
#        target:  {launch_command, stop_command, ready_check_command}
#                 (exact text simulator_launch_targets.yaml already authors
#                 for the currently-selected target, after device_bridge_
#                 host/port substitution -- only meaningful when
#                 control.desired_target is non-empty)
#      This watcher polls it every tick, and whenever control.last_updated
#      moves and control.desired_target differs from whatever is currently
#      running, stops the old one (using ITS OWN remembered stop_command --
#      see _stopDeployTarget's own comment for why that can't just be
#      re-read from the file) and starts the new one. Writes back:
#        status: {running_target, state (idle|launching|running|failed|
#                  exited), ready, pid, last_error, service_last_seen}
#      ready is populated by periodically re-running target.ready_check_command
#      once state == "running" -- polled by the device the same way it already
#      polls a target's own ready_check_command over the SSH path, just
#      reading a field instead of running a command itself.
#
#   2. cmd_<request_id>.json / status_<request_id>.json -- one-shot, request-
#      scoped actions that are NOT part of the continuous "should this be
#      running" state: install_command and check_installed_command. Install
#      itself is expected to normally run through a separate, one-time setup
#      script an operator runs by hand on this machine (not triggered by the
#      device at all) -- this path still exists for check_installed's own
#      background sweep (so the RUI can show real installed/not_installed
#      status) and as a manual escape hatch, not as the primary deploy path.
#        - Device writes  cmd_<request_id>.json    {action, target_key, script,
#                                                     timeout_sec}
#        - This watcher picks up any cmd_*.json it finds, deletes it (so it's
#          processed exactly once), and writes/updates
#                     status_<request_id>.json {target_key, action, state,
#                                                pid, exit_code, error,
#                                                updated_at}
#
#   Both share  watcher_heartbeat.json, written every HEARTBEAT_INTERVAL_SEC,
#   purely so the device side can tell "a watcher is actually alive and
#   polling this mailbox" apart from "nothing has ever run here" -- the
#   shared_storage equivalent of an SSH probe.
#
# Command/script text is the EXACT text simulator_launch_targets.yaml already
# authors for launch_command/stop_command/install_command/
# check_installed_command/ready_check_command (a `bash -lc '<script>'`
# string, after device_bridge_host/port substitution) -- this watcher strips
# that same wrapper and stages the body to a local temp file before running
# it, exactly like simulator_launcher.py's own _stage_launch_script does for
# the SSH path (see that method's docstring for why: a very long inline
# `bash -c` string handed to a fresh process is intermittently unreliable, a
# file is not). Nothing about the authored commands themselves needs to
# differ between the SSH and shared_storage transports.
#
# Deliberately zero nepi_sdk/rospy dependency -- this runs on an arbitrary
# operator machine that may not have ROS installed at all, only Python 3
# plus PyYAML (already a dependency of os_instance_registry.py/
# simulator_launcher.py on the device side, so not a new addition to this
# app's own footprint).
#
# Single-mailbox-at-a-time by design, not per-request-concurrent: this
# app's own existing convention is that only one simulator ever runs at a
# time (see SIM_OS_INSTANCES_PLAN.md's "explicitly not doing" section), so
# a launch/stop for one target_key and an install-check for another
# happening at the literal same instant is not a real scenario this needs
# to optimize for -- request-scoped files still keep concurrent CHECK
# requests (e.g. startInstalledCheckAll() checking several targets) from
# clobbering each other's results, which is the actual concurrency this
# protocol needs to survive.

import argparse
import glob
import json
import os
import subprocess
import time

import yaml

POLL_INTERVAL_SEC = 1.0
HEARTBEAT_INTERVAL_SEC = 5.0
LAUNCH_STARTUP_GRACE_SEC = 5
DEPLOY_STATE_FILENAME = 'deploy_state.yaml'
READY_CHECK_INTERVAL_SEC = 3.0

# Mirrors simulator_launcher.py's own _LAUNCH_SCRIPT_WRAPPER_PREFIX/SUFFIX --
# duplicated, not imported, so this script has zero dependency on the app's
# own Python package (it may run on a machine with no NEPI code installed
# at all beyond this one file).
_SCRIPT_WRAPPER_PREFIX = "bash -lc '"
_SCRIPT_WRAPPER_SUFFIX = "'"


def _strip_wrapper(script_text):
  if (script_text.startswith(_SCRIPT_WRAPPER_PREFIX) and
      script_text.endswith(_SCRIPT_WRAPPER_SUFFIX)):
    return script_text[len(_SCRIPT_WRAPPER_PREFIX):-len(_SCRIPT_WRAPPER_SUFFIX)]
  return script_text


def _write_json_atomic(path, data):
  # Atomic replace so the device side never reads a half-written file --
  # same reasoning as config_mgr's own YAML writes elsewhere in this
  # platform, just for JSON here.
  tmp = path + '.tmp'
  with open(tmp, 'w') as f:
    json.dump(data, f)
  os.replace(tmp, path)


class Watcher(object):
  """One instance per running vm_command_watcher.py process, one process
  per registered shared_storage OS instance."""

  def __init__(self, storage_root, instance_id):
    self.mailbox = os.path.join(storage_root, 'databases',
                                 'nepi_app_sim_connector', 'vm_commands',
                                 instance_id)
    os.makedirs(self.mailbox, exist_ok=True)
    self.work_dir = os.path.join(self.mailbox, 'work')
    os.makedirs(self.work_dir, exist_ok=True)
    # target_key -> {'proc': Popen, 'request_id': str} -- tracks the ONE
    # currently-launched process per target, so a repeat "is it still
    # running" poll (a fresh check_installed/launch-status request) can be
    # answered without re-running anything, mirroring simulator_launcher.py's
    # own self._launch_procs.
    self.launch_procs = {}
    self.processed_request_ids = set()
    # deploy_state.yaml's own tracked state -- see _pollDeployState's own
    # comment for why deploy_running_stop_command is remembered here rather
    # than re-read from the file at stop time (the file's own 'target'
    # section may already describe a DIFFERENT, newly-desired target by
    # then).
    self.deploy_proc = None
    self.deploy_running_target = ''
    self.deploy_running_stop_command = ''
    self.deploy_last_seen_update = None
    self.deploy_last_ready_check = 0.0

  def _status_path(self, request_id):
    return os.path.join(self.mailbox, 'status_' + request_id + '.json')

  def _heartbeat_path(self):
    return os.path.join(self.mailbox, 'watcher_heartbeat.json')

  def _deploy_state_path(self):
    return os.path.join(self.mailbox, DEPLOY_STATE_FILENAME)

  def run(self):
    print('vm_command_watcher: watching ' + self.mailbox)
    last_heartbeat = 0.0
    while True:
      now = time.time()
      if now - last_heartbeat > HEARTBEAT_INTERVAL_SEC:
        _write_json_atomic(self._heartbeat_path(),
                            {'alive_at': now, 'pid': os.getpid()})
        last_heartbeat = now
      self._refreshLaunchStates()
      self._checkForCommands()
      self._pollDeployState()
      time.sleep(POLL_INTERVAL_SEC)

  def _refreshLaunchStates(self):
    # Long-running launches keep their status file updated even with no
    # new incoming command -- the device polls is_ready()/launcher_status
    # continuously, not just once right after launching.
    for target_key, entry in list(self.launch_procs.items()):
      proc = entry['proc']
      rc = proc.poll()
      if rc is not None:
        _, stderr = proc.communicate()
        state = 'exited' if rc == 0 else 'failed'
        self._writeStatus(entry['request_id'], target_key, 'launch', state,
                           exit_code=rc, error=(stderr or '').strip())
        del self.launch_procs[target_key]

  def _checkForCommands(self):
    for path in sorted(glob.glob(os.path.join(self.mailbox, 'cmd_*.json'))):
      request_id = os.path.basename(path)[len('cmd_'):-len('.json')]
      try:
        with open(path, 'r') as f:
          cmd = json.load(f)
      except (OSError, ValueError):
        # Still mid-write by the device side -- try again next tick rather
        # than deleting a command this watcher never actually saw.
        continue
      # Delete first, THEN process -- a command is consumed exactly once
      # even if this watcher crashes mid-handling (the alternative, delete
      # after processing, would silently re-run a launch/stop on restart).
      try:
        os.remove(path)
      except OSError:
        pass
      if request_id in self.processed_request_ids:
        continue
      self.processed_request_ids.add(request_id)
      self._handleCommand(request_id, cmd)

  def _handleCommand(self, request_id, cmd):
    action = cmd.get('action', '')
    target_key = cmd.get('target_key', '')
    script_text = cmd.get('script', '')
    timeout_sec = cmd.get('timeout_sec', 30)
    script_path = os.path.join(self.work_dir, 'nepi_watcher_' + request_id + '.sh')
    try:
      with open(script_path, 'w') as f:
        f.write(_strip_wrapper(script_text))
    except OSError as e:
      self._writeStatus(request_id, target_key, action, 'failed',
                         error='Could not stage command script: ' + str(e))
      return

    if action == 'launch':
      self._handleLaunch(request_id, target_key, script_path)
    elif action == 'stop':
      self._handleOneShot(request_id, target_key, action, script_path, timeout_sec)
      # Mark the ORIGINAL launch's own status file (not just this stop
      # request's) as no longer running too -- otherwise a device-side
      # poller still reading status_<the launch's own request_id>.json
      # (the file it's been polling all along for "is it still up") would
      # see a stale "running" forever, since popping launch_procs here
      # means _refreshLaunchStates() never gets a chance to notice this
      # process actually died.
      launched = self.launch_procs.pop(target_key, None)
      if launched is not None:
        self._writeStatus(launched['request_id'], target_key, 'launch', 'exited')
    elif action in ('install', 'check_installed', 'ready_check', 'push_dimensions', 'kill_all'):
      # 'kill_all' is simulator_launcher.py's kill_all_gazebo() escape-hatch
      # fallback -- a one-shot blunt pkill script, same shape as the other
      # actions here, added (2026-09-08) so that button works over the
      # shared-storage transport too instead of only over a live reverse
      # SSH tunnel (see _try_shared_storage_fallback's own docstring).
      self._handleOneShot(request_id, target_key, action, script_path, timeout_sec)
    else:
      self._writeStatus(request_id, target_key, action, 'failed',
                         error='Unknown action: ' + str(action))

  def _handleLaunch(self, request_id, target_key, script_path):
    existing = self.launch_procs.get(target_key)
    if existing is not None and existing['proc'].poll() is None:
      self._writeStatus(request_id, target_key, 'launch', 'failed',
                         error=target_key + ' is already running')
      return
    # start_new_session=True (setsid) so this script's own $$ is genuinely
    # its process GROUP id too, not just its pid -- found live (2026-09-04)
    # that stop_command's own `kill -- -$(cat pgid_file)` was a silent
    # no-op without this: a plain subprocess.Popen child inherits ITS
    # PARENT's (this watcher's) process group, so the pgid file recorded a
    # number that named no real process group at all, and gzserver/the
    # rest of the launched tree survived a "successful" stop untouched.
    proc = subprocess.Popen(['bash', '-l', script_path],
                             stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True,
                             start_new_session=True)
    self.launch_procs[target_key] = {'proc': proc, 'request_id': request_id}
    time.sleep(LAUNCH_STARTUP_GRACE_SEC)
    rc = proc.poll()
    if rc is not None:
      _, stderr = proc.communicate()
      self._writeStatus(request_id, target_key, 'launch', 'failed',
                         exit_code=rc, error=(stderr or '').strip())
      del self.launch_procs[target_key]
    else:
      self._writeStatus(request_id, target_key, 'launch', 'running', pid=proc.pid)

  def _handleOneShot(self, request_id, target_key, action, script_path, timeout_sec):
    try:
      result = subprocess.run(['bash', '-l', script_path], capture_output=True,
                               text=True, timeout=timeout_sec)
      rc = result.returncode
      output = ((result.stdout or '') + (result.stderr or '')).strip()
    except subprocess.TimeoutExpired:
      rc = None
      output = 'Timed out after ' + str(timeout_sec) + 's'
    state = 'exited' if rc == 0 else 'failed'
    self._writeStatus(request_id, target_key, action, state,
                       exit_code=rc, error=('' if rc == 0 else output))

  def _writeStatus(self, request_id, target_key, action, state, pid=None,
                    exit_code=None, error=''):
    _write_json_atomic(self._status_path(request_id), {
        'request_id': request_id,
        'target_key': target_key,
        'action': action,
        'state': state,
        'pid': pid,
        'exit_code': exit_code,
        'error': error,
        'updated_at': time.time(),
    })

  #**********************
  # deploy_state.yaml -- the real deploy/kill path. See this file's own
  # module docstring for the full protocol; the short version is: one
  # persistent file per instance, control.desired_target says what SHOULD
  # be running, status.* says what actually is.

  def _readDeployState(self):
    try:
      with open(self._deploy_state_path(), 'r') as f:
        return yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
      return {}

  def _pollDeployState(self):
    state = self._readDeployState()
    if not state:
      return
    control = state.get('control') or {}
    target = state.get('target') or {}
    desired_target_key = (control.get('desired_target') or '').strip()
    last_updated = control.get('last_updated')

    # Reap a process that exited on its own (crash, or its own natural
    # end) even with no new command -- mirrors _refreshLaunchStates' own
    # reasoning for the cmd_id/status_id path.
    if self.deploy_proc is not None and self.deploy_proc.poll() is not None:
      rc = self.deploy_proc.returncode
      _, stderr = self.deploy_proc.communicate()
      self._writeDeployStatus(self.deploy_running_target,
                               'exited' if rc == 0 else 'failed',
                               error=('' if rc == 0 else (stderr or '').strip()))
      self.deploy_proc = None
      self.deploy_running_target = ''
      self.deploy_running_stop_command = ''

    command_is_new = (last_updated != self.deploy_last_seen_update)
    if command_is_new:
      self.deploy_last_seen_update = last_updated
      if desired_target_key != self.deploy_running_target:
        # Stop whatever's currently running FIRST, using the stop_command
        # remembered from when THAT target was launched -- not target
        # above, which by now already describes the NEWLY-desired target
        # (or is empty, if the device is just asking to stop).
        if self.deploy_proc is not None or self.deploy_running_target:
          self._stopDeployTarget()
        if desired_target_key:
          self._startDeployTarget(desired_target_key, target)
        else:
          self._writeDeployStatus('', 'idle')

    # Periodic readiness probe once something is actually running -- lets
    # the device poll is_ready() by reading status.ready instead of
    # dispatching its own one-shot ready_check request.
    if self.deploy_proc is not None:
      self._maybeCheckDeployReady(target)

  def _stopDeployTarget(self):
    stop_command = self.deploy_running_stop_command
    if stop_command:
      script_path = os.path.join(self.work_dir, 'nepi_deploy_stop.sh')
      try:
        with open(script_path, 'w') as f:
          f.write(_strip_wrapper(stop_command))
        subprocess.run(['bash', '-l', script_path], timeout=15)
      except Exception:
        pass
    if self.deploy_proc is not None:
      try:
        self.deploy_proc.wait(timeout=10)
      except subprocess.TimeoutExpired:
        self.deploy_proc.kill()
        self.deploy_proc.wait()
    self.deploy_proc = None
    self.deploy_running_target = ''
    self.deploy_running_stop_command = ''

  def _startDeployTarget(self, target_key, target):
    launch_command = target.get('launch_command', '')
    if not launch_command:
      self._writeDeployStatus(target_key, 'failed', error='No launch_command provided')
      return
    script_path = os.path.join(self.work_dir, 'nepi_deploy_launch_' + target_key + '.sh')
    try:
      with open(script_path, 'w') as f:
        f.write(_strip_wrapper(launch_command))
      # start_new_session=True -- see _handleLaunch's own comment on this
      # exact same line for why: without it, this script's own $$ (what it
      # echoes into its pgid file) isn't really its process group id, and
      # stop_command's `kill -- -$(cat pgid_file)` silently kills nothing.
      proc = subprocess.Popen(['bash', '-l', script_path],
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True,
                               start_new_session=True)
    except Exception as e:
      self._writeDeployStatus(target_key, 'failed', error=str(e))
      return
    self.deploy_proc = proc
    self.deploy_running_target = target_key
    self.deploy_running_stop_command = target.get('stop_command', '')
    self.deploy_last_ready_check = 0.0
    self._writeDeployStatus(target_key, 'launching', pid=proc.pid)
    time.sleep(LAUNCH_STARTUP_GRACE_SEC)
    if proc.poll() is not None:
      _, stderr = proc.communicate()
      self._writeDeployStatus(target_key, 'failed', error=(stderr or '').strip())
      self.deploy_proc = None
      self.deploy_running_target = ''
      self.deploy_running_stop_command = ''
    else:
      self._writeDeployStatus(target_key, 'running', pid=proc.pid)

  def _maybeCheckDeployReady(self, target):
    now = time.time()
    if now - self.deploy_last_ready_check < READY_CHECK_INTERVAL_SEC:
      return
    self.deploy_last_ready_check = now
    ready_check_command = target.get('ready_check_command', '')
    ready = True if not ready_check_command else False
    if ready_check_command:
      script_path = os.path.join(self.work_dir, 'nepi_deploy_ready_check.sh')
      try:
        with open(script_path, 'w') as f:
          f.write(_strip_wrapper(ready_check_command))
        result = subprocess.run(['bash', '-l', script_path], capture_output=True, timeout=10)
        ready = (result.returncode == 0)
      except Exception:
        ready = False
    self._writeDeployStatus(self.deploy_running_target, 'running', ready=ready,
                             pid=(self.deploy_proc.pid if self.deploy_proc else 0))

  def _writeDeployStatus(self, running_target, state, pid=0, error='', ready=None):
    state_doc = self._readDeployState()
    status = state_doc.get('status') or {}
    status['running_target'] = running_target
    status['state'] = state
    status['pid'] = pid
    status['last_error'] = error
    status['service_last_seen'] = time.time()
    if ready is not None:
      status['ready'] = ready
    state_doc['status'] = status
    path = self._deploy_state_path()
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
      yaml.safe_dump(state_doc, f, default_flow_style=False, sort_keys=False)
    os.replace(tmp, path)


def main():
  parser = argparse.ArgumentParser(description=(
      "Watches this OS instance's command mailbox under a locally-mounted "
      "nepi_storage share and runs launch/stop/install/check commands "
      "locally -- no SSH, no listening port. See this file's own module "
      "docstring for the full protocol."))
  parser.add_argument('--storage-root', required=True,
                       help='Local path where nepi_storage is mounted (e.g. /mnt/nepi_storage)')
  parser.add_argument('--instance-id', required=True,
                       help="This OS instance's id, matching os_instance_registry.py")
  args = parser.parse_args()
  Watcher(args.storage_root, args.instance_id).run()


if __name__ == '__main__':
  main()
