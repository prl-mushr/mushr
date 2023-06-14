#!/bin/bash
set -e

SRC_DIR="$(pwd)"

echo "Running Smoke Test"

# Build
export DOCKER_BUILDKIT=1
echo "(*) Installing @devcontainer/cli"
npm install -g @devcontainers/cli

echo "Building Dev Container"
ID_LABEL="test-container=mushr"
devcontainer up --id-label ${ID_LABEL} --workspace-folder "${SRC_DIR}"

# Test
devcontainer exec --workspace-folder "${SRC_DIR}" --id-label ${ID_LABEL} /bin/sh -c './src/mushr/.devcontainer/smoke_test.sh'

# Clean up
docker rm -f $(docker container ls -f "label=${ID_LABEL}" -q)
