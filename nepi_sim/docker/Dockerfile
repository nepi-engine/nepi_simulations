# syntax=docker/dockerfile:1

##
## Copyright (c) 2024 Numurus <https://www.numurus.com>.
##
## This file is part of nepi setup tools (nepi_setup) repo
## (see https://github.com/nepi-engine/nepi_setup)
##
## License: nepi setup tools are licensed under the "Numurus Software License",
## which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
##
## Redistributions in source code must retain this top-level comment block.
## Plagiarizing this software to sidestep the license obligations is illegal.
##
## Contact Information:
## ====================
## - mailto:nepi@numurus.com
##

# The NEPI simulator image (numurusinc/nepi:nepi-sim-latest). Everything that
# needs Ubuntu 20.04 lives here -- Gazebo Classic 11, ROS Noetic, ArduPilot
# SITL (Copter-4.5) and MuJoCo 3.2.3 -- so the host only needs Docker, the
# nepi_storage CIFS mount and the nepi_simulators commands. One container runs
# one simulator service: the entrypoint argument is "gazebo" or "mujoco".
#
# Build from the repo root (the build context must contain nepi_gazebo/ and
# nepi_mujoco/):
#   bash nepi_sim/docker/build_nepi_sim_image.sh
#
# These steps mirror nepi_gazebo_setup.sh and nepi_mujoco_setup.sh, which stay
# the reference for a native 20.04 sim machine. They are written out here as
# Docker layers rather than by running those scripts, because the scripts are
# interactive (license prompt, sudo) and depend on nepi_setup's remote bash
# environment, and because separate layers let a change to the NEPI scripts
# rebuild in seconds instead of rebuilding ArduPilot.
#
# Base image: the official ROS Noetic image rather than plain ubuntu:20.04. Its
# ROS apt source and signing key are kept current upstream. The ros.asc key
# that nepi_gazebo_setup.sh adds with apt-key expired on 2025-06-01, so that
# route no longer works on a fresh focal install.

FROM ros:noetic-ros-base-focal

ARG DEBIAN_FRONTEND=noninteractive
ARG SIM_USER=nepisim
ARG SIM_UID=1000
ARG YQ_VERSION=v4.16.2
ARG ARDUPILOT_BRANCH=Copter-4.5
ARG ARDUPILOT_GAZEBO_REF=master
ARG MUJOCO_VERSION=3.2.3

SHELL ["/bin/bash", "-o", "pipefail", "-c"]


####################################
# 1. System packages shared by both simulators, plus the GL/EGL user-space
# libraries both render through. libglvnd (libgl1/libegl1) is what the NVIDIA
# Container Toolkit plugs the host driver into; Mesa (libgl1-mesa-dri) serves
# Intel and AMD through /dev/dri. mesa-utils provides glxinfo, which the
# entrypoint uses to report which renderer the container actually got.

# hadolint ignore=DL3008
RUN apt-get update && apt-get install -y --no-install-recommends \
        sudo git wget curl ca-certificates gnupg lsb-release \
        rsync procps psmisc iproute2 iputils-ping less \
        build-essential cmake pkg-config \
        python3-pip python3-yaml python3-numpy python3-opencv python3-testresources \
        libgl1 libegl1 libgles2 libglvnd0 libglx0 libgl1-mesa-dri mesa-utils \
        libx11-6 libxext6 libxrender1 libxrandr2 libxinerama1 libxcursor1 libxi6 libxkbcommon0 \
    && rm -rf /var/lib/apt/lists/*

# mikefarah's yq v4 (the Go binary, not the apt "yq"), the version the
# services' env()/strenv() expressions are written against.
RUN arch="$(dpkg --print-architecture)" \
    && wget -q "https://github.com/mikefarah/yq/releases/download/${YQ_VERSION}/yq_linux_${arch}" -O /usr/bin/yq \
    && chmod +x /usr/bin/yq \
    && yq --version


####################################
# 2. Gazebo 11 from the OSRF repo (11.15.1, the release the Gazebo service's
# notes were measured against) and the Gazebo ROS plugins the rover drives and
# sees through. Recommends are kept, matching the native install.

# hadolint ignore=DL3008,DL3015
RUN wget -qO - https://packages.osrfoundation.org/gazebo.key \
        | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable focal main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update \
    && apt-get install -y gazebo11 libgazebo11-dev ros-noetic-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*


####################################
# 3. The simulator user. ArduPilot's prereqs script refuses to run as root,
# and the NEPI scripts call sudo for their storage syncs, so the user gets
# passwordless sudo inside the container. UID 1000 is what the sync scripts
# chown the share-side folders to. (The build steps below that run as this
# user use that sudo too, deliberately, for the same ArduPilot reason.)

RUN useradd --create-home --uid "${SIM_UID}" --shell /bin/bash "${SIM_USER}" \
    && echo "${SIM_USER} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${SIM_USER}" \
    && chmod 0440 "/etc/sudoers.d/${SIM_USER}" \
    && mkdir -p /mnt/nepi_share_storage \
    && chown "${SIM_UID}:${SIM_UID}" /mnt/nepi_share_storage

USER ${SIM_USER}
WORKDIR /home/${SIM_USER}
ENV HOME=/home/${SIM_USER} \
    USER=${SIM_USER} \
    PATH=/home/${SIM_USER}/.local/bin:/home/${SIM_USER}/ardupilot/Tools/autotest:${PATH}


####################################
# 4. ArduPilot SITL (ArduCopter, Copter-4.5 for focal's Python 3.8).
#
# The SKIP_/DO_ variables are the ones ArduPilot's own Dockerfile sets: no STM
# toolchain, no GUI extras (the Gazebo service runs sim_vehicle.py with DISPLAY
# unset), no coverage tools. As in nepi_gazebo_setup.sh, a non-zero exit from
# the prereqs script is not trusted either way: its pip phase can fail without
# changing the exit code, and with `set -e` an earlier apt failure stops it
# before its pip phase runs at all. So every Python module the SITL build and
# sim_vehicle.py need -- the script's own focal list, which includes dronecan
# for the DroneCAN code the build generates -- is checked and repaired
# afterwards, and the build fails here, by name, if one is still missing.

# hadolint ignore=DL3003
RUN git clone --branch "${ARDUPILOT_BRANCH}" https://github.com/ArduPilot/ardupilot.git ardupilot \
    && cd ardupilot \
    && git submodule update --init --recursive

# hadolint ignore=DL3003,DL3004
RUN cd ardupilot \
    && { SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1 DO_AP_STM_ENV=0 \
         Tools/environment_install/install-prereqs-ubuntu.sh -y \
         || echo "WARNING: install-prereqs-ubuntu.sh exited non-zero -- checking modules below"; } \
    && sudo rm -rf /var/lib/apt/lists/*

# Word splitting of ${missing} is intended. Versions follow the native setup.
# hadolint ignore=SC2086,DL3013
RUN missing=""; \
    for pair in em:empy==3.3.4 pymavlink:pymavlink MAVProxy:MAVProxy \
                serial:pyserial future:future lxml:lxml \
                dronecan:dronecan pexpect:pexpect ptyprocess:ptyprocess geocoder:geocoder; do \
        python3 -c "import ${pair%%:*}" >/dev/null 2>&1 || missing="${missing} ${pair#*:}"; \
    done; \
    if [[ -n "$missing" ]]; then \
        echo "Installing missing SITL Python modules:${missing}"; \
        python3 -m pip install --user --no-cache-dir ${missing}; \
    fi; \
    for module in em pymavlink MAVProxy serial future lxml dronecan pexpect ptyprocess geocoder; do \
        python3 -c "import ${module}" || { echo "ERROR: Python module ${module} is still missing"; exit 1; }; \
    done

# On failure, print the end of waf's own log: the reason waf gives on the
# console is usually just "see config.log".
# hadolint ignore=DL3003
RUN cd ardupilot \
    && { ./waf configure --board sitl \
         || { echo "---- build/config.log (last 80 lines) ----"; tail -n 80 build/config.log; exit 1; }; } \
    && ./waf copter

# The copter default parameters (eeprom.bin) are baked into the image, so every
# container starts SITL from the same clean parameter set.
COPY --chown=${SIM_UID}:${SIM_UID} nepi_sim/docker/build/nepi_sim_init_sitl.sh /tmp/nepi_sim_init_sitl.sh
RUN bash /tmp/nepi_sim_init_sitl.sh && rm -f /tmp/nepi_sim_init_sitl.sh


####################################
# 5. The ArduPilot-Gazebo bridge plugin (ArduPilotPlugin), and the iris model
# the quadcopter robot config is copied from.

# hadolint ignore=DL3003,DL3004
RUN git clone https://github.com/khancyr/ardupilot_gazebo.git ardupilot_gazebo \
    && cd ardupilot_gazebo \
    && git checkout "${ARDUPILOT_GAZEBO_REF}" \
    && mkdir -p build && cd build \
    && cmake .. \
    && make -j"$(nproc)" \
    && sudo make install


####################################
# 6. MuJoCo, pinned to 3.2.3: the newest release for Python 3.8, and the MjSpec
# API nepi_mujoco_sim.py is written against. Installed per user, as natively.

RUN python3 -m pip install --user --no-cache-dir "mujoco==${MUJOCO_VERSION}" \
    && python3 -c "import mujoco; print('mujoco', mujoco.__version__)"


####################################
# 7. The NEPI simulation scripts and resources. These are the layers that
# change most, so they come last. The scripts run from /opt/nepi_simulations;
# the robot, world and config files the services read are staged into
# ~/gazebo and ~/mujoco exactly as the native setup scripts stage them, and
# the sync scripts reconcile those copies with NEPI storage on every start.

COPY --chown=${SIM_UID}:${SIM_UID} nepi_gazebo /opt/nepi_simulations/nepi_gazebo
COPY --chown=${SIM_UID}:${SIM_UID} nepi_mujoco /opt/nepi_simulations/nepi_mujoco
COPY --chown=${SIM_UID}:${SIM_UID} nepi_sim/docker/container /opt/nepi_simulations/nepi_sim/docker/container

RUN gazebo_res=/opt/nepi_simulations/nepi_gazebo/gazebo_setup/resources \
    && mujoco_res=/opt/nepi_simulations/nepi_mujoco/mujoco_setup/resources \
    && for folder in config ENVIRONMENT_CONFIGS ROBOT_CONFIGS; do \
           mkdir -p "${HOME}/gazebo/${folder}" "${HOME}/mujoco/${folder}"; \
           rsync -ar --delete "${gazebo_res}/${folder}/" "${HOME}/gazebo/${folder}/"; \
           rsync -ar --delete "${mujoco_res}/${folder}/" "${HOME}/mujoco/${folder}/"; \
       done \
    && iris="${HOME}/ardupilot_gazebo/models/iris_with_ardupilot" \
    && cp "${iris}/model.config" "${iris}/model.sdf" "${HOME}/gazebo/ROBOT_CONFIGS/quadcopter/" \
    && sed -i 's|<imuName>[^<]*</imuName>|<imuName>imu_sensor</imuName>|' \
           "${HOME}/gazebo/ROBOT_CONFIGS/quadcopter/model.sdf" \
    && echo 'source /opt/nepi_simulations/nepi_sim/docker/container/nepi_sim_container_utils' >> "${HOME}/.bashrc"

# NEPI_SIM_CONTAINER tells the mount helpers in the NEPI bash utils that the
# host owns the share mount. ROS stays on loopback: every ROS node is in this
# container, and with host networking that avoids depending on how the host's
# hostname resolves. QT_X11_NO_MITSHM keeps the Gazebo GUI off X shared memory,
# which a container cannot share with the host X server.
ENV NEPI_SIM_CONTAINER=1 \
    NEPI_MODE=REMOTE \
    NEPI_USER=nepi \
    TMPDIR=/tmp \
    ROS_HOSTNAME=localhost \
    ROS_MASTER_URI=http://localhost:11311 \
    QT_X11_NO_MITSHM=1 \
    NEPI_GAZEBO_RESOURCES_FOLDER=/opt/nepi_simulations/nepi_gazebo/gazebo_setup/resources \
    NEPI_MUJOCO_RESOURCES_FOLDER=/opt/nepi_simulations/nepi_mujoco/mujoco_setup/resources

ENTRYPOINT ["/opt/nepi_simulations/nepi_sim/docker/container/nepi_sim_entrypoint.sh"]
CMD ["help"]
