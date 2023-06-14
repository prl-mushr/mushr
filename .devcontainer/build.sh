#!/bin/bash

# This script is meant to be executed within the "postCreateCommand" field in
# .devcontainer/devcontainer.json, i.e., it expects the devcontainer config to install
# necessary folders with source code beforehand.

set -eo pipefail
set -x

cd "${HOME}"/catkin_ws/src
./mushr/mushr_utils/install/mushr_install.bash --trivial-only
mushr_noetic run 'cd catkin_ws && catkin build'
