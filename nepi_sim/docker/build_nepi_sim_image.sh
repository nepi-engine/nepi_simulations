#!/bin/bash

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

# Build (and optionally push) the nepi_sim image from this checkout. For
# maintainers: users pull the published image with nepi_simulators_pull.
#
# Usage: bash build_nepi_sim_image.sh [--push] [--tag <image:tag>] [extra docker build args]
#
# The build context is the repo root, so the image carries exactly the
# nepi_gazebo and nepi_mujoco scripts of this checkout. The first build takes
# a while (ArduPilot is built from source); a change to the NEPI scripts only
# rebuilds the last layers.

SCRIPT_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)
REPO_FOLDER=$(cd -P "${SCRIPT_FOLDER}/../.." >/dev/null 2>&1 && pwd)

IMAGE=numurusinc/nepi:nepi-sim-latest
PUSH=0
EXTRA_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --push) PUSH=1; shift ;;
        --tag) IMAGE=$2; shift 2 ;;
        *) EXTRA_ARGS+=("$1"); shift ;;
    esac
done

# Recorded in the image so a running container can say which checkout it is.
REVISION=$(git -C "$REPO_FOLDER" rev-parse --short HEAD 2>/dev/null || echo unknown)

echo "Building ${IMAGE} from ${REPO_FOLDER} (revision ${REVISION})"
DOCKER_BUILDKIT=1 docker build \
    -f "${SCRIPT_FOLDER}/Dockerfile" \
    -t "$IMAGE" \
    --label "org.opencontainers.image.revision=${REVISION}" \
    "${EXTRA_ARGS[@]}" \
    "$REPO_FOLDER" || exit 1

if [[ $PUSH -eq 1 ]]; then
    echo "Pushing ${IMAGE}"
    docker push "$IMAGE" || exit 1
fi
