#!/bin/bash
set -e

# Install caddy
export DEBIAN_FRONTEND=noninteractive
sudo apt update && sudo apt install -y debian-keyring debian-archive-keyring apt-transport-https
curl -1sLf 'https://dl.cloudsmith.io/public/caddy/stable/gpg.key' | sudo gpg --dearmor -o /usr/share/keyrings/caddy-stable-archive-keyring.gpg
curl -1sLf 'https://dl.cloudsmith.io/public/caddy/stable/debian.deb.txt' | sudo tee /etc/apt/sources.list.d/caddy-stable.list
sudo apt update && sudo apt install -y caddy

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