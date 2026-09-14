#!/bin/bash
# Keeps /mnt/nepi_share_storage mounted to the NEPI device's nepi_storage CIFS
# share. Run as a persistent systemd service (Restart=always), not a one-shot
# mount: this loops forever, checking every CHECK_INTERVAL_SEC whether the
# mount is still live and (re)mounting it if not -- so it recovers from the
# device rebooting, a network drop, or this VM coming up before the device is
# reachable yet, the same "just keep trying" resilience autossh used to give
# the reverse tunnel, now applied to the shared-storage side instead.
set -u

DEVICE_HOST="${NEPI_DEVICE_SSH_HOST:-192.168.179.103}"
SHARE="//${DEVICE_HOST}/nepi_storage"
MOUNT_POINT="/mnt/nepi_share_storage"
CREDS_FILE="/etc/nepi_storage_credentials"
MOUNT_OPTS="credentials=${CREDS_FILE},uid=2000,gid=2000,vers=3.1.1"
CHECK_INTERVAL_SEC=10

mkdir -p "$MOUNT_POINT"

while true; do
  if ! mountpoint -q "$MOUNT_POINT"; then
    echo "$(date -Is) mounting ${SHARE} -> ${MOUNT_POINT}"
    if mount -t cifs "$SHARE" "$MOUNT_POINT" -o "$MOUNT_OPTS"; then
      echo "$(date -Is) mount succeeded"
    else
      echo "$(date -Is) mount failed, retrying in ${CHECK_INTERVAL_SEC}s"
    fi
  fi
  sleep "$CHECK_INTERVAL_SEC"
done
