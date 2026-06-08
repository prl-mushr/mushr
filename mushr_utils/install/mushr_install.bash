#!/bin/bash
# MuSHR jazzy installer.
# Mirrors the structure of the original ROS 1 mushr_install.bash, adapted
# for ROS 2 jazzy + the additional dependencies (range_libc, librealsense
# realsenseai repo, YDLidar SDK, custom rosdep keys).

pushd "$(dirname "$0")" > /dev/null

# Are we in the right place?
if [[ ! -f mushr_install.bash ]]; then
    echo "Wrong directory! Change directory to the one containing mushr_install.bash"
    exit 1
fi

# Detect OS / arch
export MUSHR_OS_TYPE="$(uname -m)"
export MUSHR_INSTALL_PATH="$(pwd)"
# MUSHR_WS_PATH points to the parent of colcon_ws/. Override via env if your
# layout differs from the default ~/colcon_ws.
export MUSHR_WS_PATH="${MUSHR_WS_PATH:-${HOME}}"
mkdir -p "${MUSHR_WS_PATH}/colcon_ws/src"

# Install vcstool on host (needed for vcs import below)
if ! command -v vcs >/dev/null 2>&1; then
    echo "Installing python3-vcstool..."
    sudo apt-get update
    sudo apt-get install -y python3-vcstool
fi

# Pull workspace dependencies declared in base-repos.yaml
# (nav-repos.yaml is intentionally NOT imported here — install that manually.)
SRC_DIR="${MUSHR_WS_PATH}/colcon_ws/src"
BASE_REPOS_YAML="${MUSHR_INSTALL_PATH}/../../base-repos.yaml"
if [[ -f "${BASE_REPOS_YAML}" ]]; then
    echo "Importing base-repos.yaml dependencies into ${SRC_DIR}..."
    (cd "${SRC_DIR}" && vcs import < "${BASE_REPOS_YAML}")
else
    echo "WARNING: ${BASE_REPOS_YAML} not found — skipping vcs import."
fi

# Real robot vs sim
read -p "Are you installing on the robot and need all the sensor drivers? (y/n) " -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    export MUSHR_REAL_ROBOT=1
    export MUSHR_COMPOSE_FILE=docker-compose-robot.yml
else
    export MUSHR_REAL_ROBOT=0
    export MUSHR_COMPOSE_FILE=docker-compose-cpu.yml
fi

# Build vs pull
read -p "Build from scratch? (Not recommended, takes much longer than pulling ready-made image) (y/n) " -r
echo
export BUILD_FROM_SCRATCH=0
if [[ $REPLY =~ ^[Yy]$ ]]; then
    export BUILD_FROM_SCRATCH=1
    if [[ $MUSHR_REAL_ROBOT == 1 ]]; then
        export MUSHR_COMPOSE_FILE=docker-compose-build-robot.yml
    else
        export MUSHR_COMPOSE_FILE=docker-compose-build-cpu.yml
    fi
fi

if [ "$MUSHR_OS_TYPE" = "x86_64" ]; then
    export MUSHR_BASE_IMAGE="nvcr.io/nvidia/isaac/ros:isaac_ros_28556f8bc78a98822bd08b2d7c6fcf9b-amd64"
elif [ "$MUSHR_OS_TYPE" = "aarch64" ]; then
    export MUSHR_BASE_IMAGE="nvcr.io/nvidia/isaac/ros:isaac_ros_28556f8bc78a98822bd08b2d7c6fcf9b-arm64-jetpack"
else
    echo "Unknown OS type: $MUSHR_OS_TYPE"
    exit 0
fi


# Robot-side host setup (only on real robot)
if [[ $MUSHR_REAL_ROBOT == 1 ]]; then
    echo "Running robot-specific host setup..."

    # Ensure user can run docker without sudo
    sudo usermod -aG docker "$USER" || true

    # Ensure docker compose v2 plugin is available
    if ! docker compose version >/dev/null 2>&1; then
        echo "Installing docker compose v2 plugin..."
        sudo apt-get update
        sudo apt-get install -y docker-compose-plugin || {
            # Fallback: download plugin binary directly
            DOCKER_CONFIG=${DOCKER_CONFIG:-$HOME/.docker}
            mkdir -p "${DOCKER_CONFIG}/cli-plugins"
            ARCH=$(uname -m)
            case "${ARCH}" in
                aarch64) DC_ARCH=aarch64 ;;
                x86_64)  DC_ARCH=x86_64 ;;
                *) echo "Unsupported arch: ${ARCH}"; exit 1 ;;
            esac
            curl -SL "https://github.com/docker/compose/releases/latest/download/docker-compose-linux-${DC_ARCH}" \
                -o "${DOCKER_CONFIG}/cli-plugins/docker-compose"
            chmod +x "${DOCKER_CONFIG}/cli-plugins/docker-compose"
        }
    fi

    # Register nvidia container runtime if missing
    if ! docker info 2>/dev/null | grep -qi 'Runtimes:.*nvidia'; then
        echo "Registering nvidia container runtime..."
        if ! command -v nvidia-ctk >/dev/null 2>&1; then
            sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
        fi
        sudo nvidia-ctk runtime configure --runtime=docker
        sudo systemctl restart docker
    fi

    # VESC udev rule
    echo 'ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="vesc"' \
        | sudo tee /etc/udev/rules.d/10-vesc.rules > /dev/null

    # Jetson GPIO udev + group
    sudo groupadd -f -r gpio
    sudo usermod -a -G gpio "$USER" || true
    sudo wget -q https://raw.githubusercontent.com/NVIDIA/jetson-gpio/master/lib/python/Jetson/GPIO/99-gpio.rules \
        -O /etc/udev/rules.d/99-gpio.rules

    sudo udevadm control --reload-rules && sudo udevadm trigger
fi

# Build or pull
if [[ $BUILD_FROM_SCRATCH == 1 ]]; then
    echo "Building docker image from scratch (this can take a while)..."
    docker compose -f "${MUSHR_INSTALL_PATH}/${MUSHR_COMPOSE_FILE}" build
else
    echo "Pulling pre-built docker image..."
    docker compose -f "${MUSHR_INSTALL_PATH}/${MUSHR_COMPOSE_FILE}" pull || {
        echo "Pull failed. Falling back to build from scratch..."
        if [[ $MUSHR_REAL_ROBOT == 1 ]]; then
            export MUSHR_COMPOSE_FILE=docker-compose-build-robot.yml
        else
            export MUSHR_COMPOSE_FILE=docker-compose-build-cpu.yml
        fi
        docker compose -f "${MUSHR_INSTALL_PATH}/${MUSHR_COMPOSE_FILE}" build
    }
fi

# If sim, mark hardware-only packages with COLCON_IGNORE so colcon skips them
if [[ $MUSHR_REAL_ROBOT == 0 ]]; then
    for ignored_package in push_button_utils ydlidar_ros2_driver realsense-ros; do
        target="${MUSHR_WS_PATH}/colcon_ws/src/mushr/mushr_hardware/${ignored_package}"
        [ -d "${target}" ] && touch "${target}/COLCON_IGNORE"
    done
fi

# Generate the mushr_jazzy launcher
cat > "${MUSHR_INSTALL_PATH}/mushr_jazzy" <<EOF
#!/bin/bash
export MUSHR_INSTALL_PATH=${MUSHR_INSTALL_PATH}
export MUSHR_REAL_ROBOT=${MUSHR_REAL_ROBOT}
export MUSHR_WS_PATH=${MUSHR_WS_PATH}
export MUSHR_COMPOSE_FILE=${MUSHR_COMPOSE_FILE}
export MUSHR_OS_TYPE=${MUSHR_OS_TYPE}
export MUSHR_BASE_IMAGE=${MUSHR_BASE_IMAGE}

NAME=mushr_jazzy

case "\${1:-run}" in
    run)
        # Already running -> exec into it
        if docker ps --format '{{.Names}}' | grep -q "^\${NAME}\$"; then
            exec docker exec -it "\${NAME}" bash
        fi
        # Exists but stopped -> start + exec (preserves apt/rosdep state)
        if docker ps -a --format '{{.Names}}' | grep -q "^\${NAME}\$"; then
            docker start "\${NAME}" > /dev/null
            exec docker exec -it "\${NAME}" bash
        fi
        # Brand-new -> create container without --rm so it persists
        xhost +local:docker > /dev/null 2>&1 || true
        exec docker compose -f "\${MUSHR_INSTALL_PATH}/\${MUSHR_COMPOSE_FILE}" \\
            run --service-ports --name "\${NAME}" mushr_jazzy bash
        ;;
    build)
        exec docker compose -f "\${MUSHR_INSTALL_PATH}/\${MUSHR_COMPOSE_FILE}" \\
            build --no-cache mushr_jazzy
        ;;
    rm|clean)
        # Wipe the persisted container so the next 'run' creates a fresh one
        docker rm -f "\${NAME}" 2>/dev/null || true
        echo "Removed container \${NAME}."
        ;;
    *)
        echo "Invalid command. Valid: 'run' (default), 'build', 'rm'"
        exit 1
        ;;
esac
EOF
chmod +x "${MUSHR_INSTALL_PATH}/mushr_jazzy"

echo "Installing mushr_jazzy launcher to /usr/local/bin..."
sudo ln -sf "${MUSHR_INSTALL_PATH}/mushr_jazzy" /usr/local/bin/mushr_jazzy

echo "Done. Run 'mushr_jazzy' to launch the container."

popd > /dev/null
