#!/bin/bash
# One-shot setup for a fresh Ubuntu machine (VM, laptop, or bare metal) to
# become a usable Sim Connector deploy target -- the "any new ubuntu os with
# an ethernet connection" bring-up path this repo's own shared-storage
# transport was designed around (see vm_command_watcher.py's own docstring:
# "a real deployment has NO device-to-VM network path at all, reverse-SSH
# included -- installation will already be dealt with in a setup script").
#
# What this does NOT do: install Gazebo/ArduPilot/Webots/MuJoCo themselves.
# That's handled automatically, per-target, the first time you Deploy from
# the Sim Connector RUI (simulator_launch_targets.yaml's own
# install_command/check_installed_command machinery) -- this script only
# gets the CONNECTION working, so that automatic install-on-first-deploy can
# actually reach this machine and run.
#
# Usage:
#   ./nepi_vm_connect_setup.sh <device_host_or_ip> [instance_id]
#
# device_host_or_ip: the NEPI device's LAN address (its static IP is the
#   safest choice -- see docs/SIM_VM_CONNECTION_SETUP.md for why a DHCP
#   interface can be the wrong one on a multi-homed device). Added to
#   /etc/hosts as "nepi" if that name doesn't already resolve, since
#   simulator_launch_targets.yaml's own device_bridge_host defaults to the
#   plain hostname "nepi".
# instance_id: this machine's own identifier under the shared-storage
#   vm_commands/ tree. Defaults to "os_$(hostname)" -- must match whatever
#   the device's own Sim Connector app has this machine registered as if
#   you've ever used the (optional, now-secondary) reverse-SSH OS-instance
#   picker; the plain shared-storage fallback described above needs no such
#   registration at all, so the default is fine for that path.
set -eu

DEVICE_HOST="${1:?Usage: $0 <device_host_or_ip> [instance_id]}"
# 'baseline' is the zero-registration default: os_instance_registry.py's
# ensure_baseline() gives every target this same os_instance_id unless an
# operator has explicitly registered/selected a different one through the
# (optional, now-secondary) reverse-SSH OS-instance picker -- so a watcher
# started with this ID is picked up automatically with no device-side
# action at all. Only pass a different instance_id here if you know this
# machine was explicitly registered under one (check the device's Sim
# Connector app state, or docs/SIM_VM_CONNECTION_SETUP.md's manual-registration
# section).
INSTANCE_ID="${2:-baseline}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUN_USER="$(id -un)"

echo "== NEPI VM connect setup =="
echo "Device host/IP : ${DEVICE_HOST}"
echo "Instance ID     : ${INSTANCE_ID}"
echo "Repo checkout   : ${REPO_ROOT}"
echo "Run as user     : ${RUN_USER}"
echo

# --- 1. /etc/hosts entry, so device_bridge_host: "nepi" resolves ---
if ! getent hosts nepi > /dev/null 2>&1; then
  echo "-- Adding /etc/hosts entry for 'nepi' -> ${DEVICE_HOST}"
  echo "${DEVICE_HOST} nepi" | sudo tee -a /etc/hosts > /dev/null
else
  echo "-- 'nepi' already resolves ($(getent hosts nepi | awk '{print $1}')), leaving /etc/hosts alone"
fi

# --- 2. cifs-utils, for mount -t cifs ---
if ! command -v mount.cifs > /dev/null 2>&1; then
  echo "-- Installing cifs-utils"
  sudo apt-get update -qq
  sudo apt-get install -y cifs-utils
else
  echo "-- cifs-utils already installed"
fi

# --- 3. CIFS credentials ---
CREDS_FILE="/etc/nepi_storage_credentials"
if [ ! -f "$CREDS_FILE" ]; then
  echo "-- ${CREDS_FILE} not found. Enter the NEPI device's nepi_storage share credentials"
  echo "   (same account used everywhere else on this device, default nepi/nepi unless changed):"
  read -rp "   username: " SMB_USER
  read -rsp "   password: " SMB_PASS
  echo
  sudo tee "$CREDS_FILE" > /dev/null <<EOF
username=${SMB_USER}
password=${SMB_PASS}
EOF
  sudo chmod 600 "$CREDS_FILE"
else
  echo "-- ${CREDS_FILE} already present, leaving it alone"
fi

# --- 4. systemd units, rendered with this machine's real paths ---
SYSTEMD_DIR="${REPO_ROOT}/sim_container/systemd"
render_unit() {
  local template="$1" dest="$2"
  sed \
    -e "s#^Environment=NEPI_DEVICE_SSH_HOST=.*#Environment=NEPI_DEVICE_SSH_HOST=${DEVICE_HOST}#" \
    -e "s#^Environment=NEPI_DRONES_REPO_PATH=.*#Environment=NEPI_DRONES_REPO_PATH=${REPO_ROOT}#" \
    -e "s#^Environment=NEPI_VM_INSTANCE_ID=.*#Environment=NEPI_VM_INSTANCE_ID=${INSTANCE_ID}#" \
    -e "s#^User=.*#User=${RUN_USER}#" \
    "$template" | sudo tee "$dest" > /dev/null
}
echo "-- Installing systemd units"
render_unit "${SYSTEMD_DIR}/nepi-storage-mount.service" /etc/systemd/system/nepi-storage-mount.service
render_unit "${SYSTEMD_DIR}/nepi-vm-command-watcher.service" /etc/systemd/system/nepi-vm-command-watcher.service
sudo systemctl daemon-reload

# --- 5. enable + start ---
echo "-- Enabling and starting services"
sudo systemctl enable --now nepi-storage-mount.service
# Give the mount a moment before starting the watcher (its own ExecStartPre
# also waits, this just avoids a guaranteed first failed start in the log).
for i in $(seq 1 15); do
  mountpoint -q /mnt/nepi_share_storage && break
  sleep 1
done
# restart, not enable --now: this script is meant to be safely re-runnable
# (e.g. after editing a unit template) -- enable --now is a no-op on an
# already-running service, silently leaving it on its OLD unit definition.
sudo systemctl enable nepi-vm-command-watcher.service
sudo systemctl restart nepi-vm-command-watcher.service

# --- 6. verify ---
echo
echo "== Verification =="
if mountpoint -q /mnt/nepi_share_storage; then
  echo "OK: /mnt/nepi_share_storage is mounted"
else
  echo "FAILED: /mnt/nepi_share_storage did not mount -- check: journalctl -u nepi-storage-mount.service"
  exit 1
fi
if systemctl is-active --quiet nepi-vm-command-watcher.service; then
  echo "OK: nepi-vm-command-watcher.service is running"
else
  echo "FAILED: nepi-vm-command-watcher.service is not running -- check: journalctl -u nepi-vm-command-watcher.service"
  exit 1
fi
echo
echo "Done. This machine (instance '${INSTANCE_ID}') is ready to be used as a Sim Connector"
echo "deploy target -- clicking Deploy in the RUI for any target now reaches it automatically,"
echo "installing Gazebo/ArduPilot/Webots/MuJoCo on first use as needed."
