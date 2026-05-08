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
git clone <MUSHR_CLASS_REPO_URL> mushr
cd ~/colcon_ws/src/mushr/mushr_utils/install
./mushr_install.bash
```

The installer will:
1. Install `python3-vcstool` if missing.
2. Run `vcs import < ../../base-repos.yaml` from `~/colcon_ws/src/` to
   pull all workspace dependencies (vesc, transport_drivers, YDLidar SDK,
   realsense-ros, push_button_utils, ydlidar_ros2_driver, etc.).
3. Prompt for **robot vs sim** — `y` on the car (installs hardware
   drivers, registers nvidia runtime, writes udev rules, adds the user
   to `gpio` group).
4. Prompt for **build vs pull** — `n` to pull the pre-built image
   (fast); `y` to build locally from the Dockerfile (~30 min on Jetson).
   If a pre-built image isn't published yet, pull will fall back to a
   local build automatically.
5. Drop `COLCON_IGNORE` into hardware-only packages on sim installs.
6. Write `mushr_humble` here and symlink it into `/usr/local/bin`.

`nav-repos.yaml` is **NOT** imported automatically — install that
repo manually if you need the class-code packages:

```bash
cd ~/colcon_ws/src
vcs import < mushr/nav-repos.yaml
```

After the container starts (run `mushr_humble`), build the workspace:

```bash
cd ~/colcon_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

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

The container is **persistent**: when you exit, it stops but isn't
removed. The next `mushr_humble` restarts the same container, so apt
packages installed by `rosdep install` stay around and you do **not**
need to rerun `rosdep install` every session. Run `mushr_humble rm`
if you ever want to wipe it and start fresh.

## `mushr_humble` subcommands

| Command | Action |
|---|---|
| `mushr_humble` or `mushr_humble run` | enter the container (creates it the first time, restarts/`exec`s thereafter) |
| `mushr_humble build` | rebuild the image with `--no-cache` |
| `mushr_humble rm` | remove the persisted container (next `run` creates a fresh one) |

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
