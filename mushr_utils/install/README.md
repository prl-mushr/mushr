# MuSHR Humble Docker

ROS 2 Humble port of the original `mushr_noetic` install pattern.
Containerized environment for the MuSHR car: ROS 2 Humble, range_libc,
librealsense, YDLidar SDK, Jetson.GPIO, and the scientific Python stack
all baked in. The workspace itself is **not** copied into the image; it is
bind-mounted at runtime so source edits on the host appear immediately
inside the container.

## Files

```
mushr_utils/install/
├── Dockerfile                         image definition; calls the install_scripts/
├── install_scripts/
│   ├── mushr_install_ros.bash         ros-humble-desktop, rosdep, custom keys
│   ├── mushr_install_deps.bash        apt extras, pip, range_libc, dev tools
│   └── mushr_install_hw_drivers.bash  librealsense, YDLidar SDK, Jetson.GPIO
├── docker-compose-cpu.yml             sim runtime (image-only)
├── docker-compose-robot.yml           robot runtime (image-only, nvidia + /dev)
├── docker-compose-build-cpu.yml       sim build
├── docker-compose-build-robot.yml     robot build
├── mushr_install.bash                 top-level installer + mushr_humble generator
└── README.md                          this file
```

## Install

```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone --recurse-submodules <MUSHR_REPO_URL> mushr
cd ~/colcon_ws/src/mushr/mushr_utils/install
./mushr_install.bash
```

Two prompts:
- **robot vs sim** — `y` on the car (installs hardware drivers, registers
  nvidia runtime, writes udev rules, adds the user to `gpio` group).
- **build vs pull** — `n` to pull the pre-built image (fast); `y` to build
  locally from the Dockerfile (~30 min on Jetson).

The installer writes `mushr_humble` into this directory and symlinks
it into `/usr/local/bin`.

After the container starts, build the workspace:

```bash
cd ~/colcon_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

On sim installs, `COLCON_IGNORE` is dropped into `mushr_hardware/`
sub-packages that need real hardware to build.

## Daily use

```bash
mushr_humble
```

Drops you into a bash shell inside the container with the workspace
mounted at `/root/colcon_ws`. The first time, build the workspace:

```bash
cd ~/colcon_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

After that, `~/.bashrc` (in the image) auto-sources both
`/opt/ros/humble/setup.bash` and `~/colcon_ws/install/setup.bash` if it
exists. Subsequent `mushr_humble` invocations just need
`source install/setup.bash` if the overlay isn't picked up.

A second terminal? Run `mushr_humble` again — the launcher detects the
running container and `docker exec`s into it.

## `mushr_humble` subcommands

| Command | Action |
|---|---|
| `mushr_humble` or `mushr_humble run` | enter the container (start fresh or `exec` into running) |
| `mushr_humble build` | rebuild the image with `--no-cache` |

## Architecture

- **Image** = read-only environment: ROS 2, range_libc, drivers, system
  deps. Pre-built once, pulled fast.
- **Bind mount** = `~/colcon_ws` ↔ `/root/colcon_ws`. Source + build
  artifacts live on the host, persist across container exits.
- **Compose matrix** = (cpu, robot) × (run, build). CPU/robot differ in
  `runtime: nvidia`, `network_mode: host`, `/dev` and tegrastats mounts.
  Run/build differ in `image:` (pull) vs `build:` (local Dockerfile).
- **Hardware install gated on `REAL=1` build arg** — sim image stays slim.
- **Host-side udev/GPIO/nvidia-ctk** done by `mushr_install.bash` (needs
  root on host kernel; container can't set them).
- **`nvidia-l4t-*`, `cuda-*`, `tensorrt`** are NOT installed in the image.
  They live on the host (JetPack) and are exposed via `--runtime=nvidia`
  through the NVIDIA Container Toolkit.

## Updating

Source edits on the host appear immediately. For pure
Python/launch/config changes, no rebuild is needed
(`--symlink-install`). For C++ changes, rerun `colcon build` inside the
container. To pick up Dockerfile or system-dep changes:

```bash
mushr_humble build
```
