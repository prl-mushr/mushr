#!/bin/bash
set -e

# Install foxglove studio
cleanup() {
    EXIT_CODE=$?
    set +e
    if [[ -n "${CONTAINER_ID}" ]]; then
        echo "Executing cleanup of the container"
        docker rm -f "$CONTAINER_ID"
    fi
    exit $EXIT_CODE
}
trap cleanup EXIT

CONTAINER_ID=$(docker run -d ghcr.io/foxglove/studio)
sudo docker cp "$CONTAINER_ID:/src" /srv/foxglove
docker stop "$CONTAINER_ID"
docker rm "$CONTAINER_ID"