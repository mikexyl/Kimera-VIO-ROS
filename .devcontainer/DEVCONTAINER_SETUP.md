# Dev Container Setup Guide

## Overview

This dev container follows a **single-stage base image** approach recommended for easier maintenance and faster development iteration. The philosophy is:

1. **Base Docker Image** (`ubuntu_base:noetic`): Contains only system-level dependencies that rarely change
2. **Catkin Workspace**: All project packages are built in the mounted workspace using `catkin build`
3. **Entrypoint**: Only sources environment scripts, does NOT build anything

This is simpler than multi-stage Docker builds and allows for quick rebuilds during development.

---

## Architecture

### What's in the Base Docker Image (`ubuntu_base:noetic`)

Built from `/workspaces/src/ros-cuda-dockers/Dockerfile`:
- Ubuntu 20.04
- ROS Noetic (ros-base)
- System development tools (build-essential, git, cmake, etc.)

Extended by `kimera-vio-ros/.devcontainer/Dockerfile`:
- **ROS perception packages**: ros-noetic-perception, rviz, realsense2-camera, etc.
- **System libraries**: spdlog, graphviz, metis, gflags, glog, googletest, libblas
- **Development tools**: taskfile, pixi, rust toolchain, python3-catkin-tools, python3-vcstool

**That's it!** The Dockerfile is intentionally minimal. All C++ dependencies are built in the catkin workspace.

### What's in the Catkin Workspace (Built at Runtime/Development)

Located at `/workspaces/src/` and mounted from your host:

**Frequently edited packages** (rebuilt often during development):
- `online_fgo_core` - Core FGO library (CMake package)
- `online_fgo_ros1` - ROS1 adapter for FGO (Catkin package)
- `Kimera-VIO` - Visual-Inertial Odometry (CMake package)
- `kimera-vio-ros` - ROS wrapper for Kimera-VIO (Catkin package)

**Supporting packages** (rarely edited):
- `aria_common` - Aria dataset utilities (CMake package)
- `aria_visualization` - Aria visualization tools (CMake package)
- `catkin_simple` - Catkin build macros
- `pose_graph_tools` - Pose graph visualization utilities

**Heavy dependencies** (built once, cached by catkin):
- `gtsam` - Factor graph optimization library v4.1.1 (with GTSAM_UNSTABLE)
- `opengv` - Geometric vision algorithms
- `dbow2` - Bag of words for place recognition
- `kimera_rpgo` - Robust pose graph optimization
- `GeographicLib` - Geographic coordinate conversions (optional)
- `rerun_sdk` - Visualization framework (optional)

**Catkin wrappers** (for system compatibility):
- `opencv3_catkin` - OpenCV 3.4+
- `eigen_catkin` - Eigen3
- `gflags_catkin` / `glog_catkin` - Google utilities

**Note:** Some catkin wrappers (dbow2_catkin, opengv_catkin) are listed in Kimera-VIO's package.xml but may not exist as separate repos. The plain CMake versions (dbow2, opengv) work fine with catkin when given a proper package.xml.

All of these are cloned into the workspace and built with `catkin build`. This keeps the Docker image simple and allows easy version updates.

---

## Package Types

### CMake Packages (Plain CMake, not Catkin)

These use `<build_type>cmake</build_type>` in their `package.xml`:
- `online_fgo_core`
- `Kimera-VIO`
- `aria_common`
- `aria_visualization`

They are built by `catkin build` but use standard CMake (no catkin macros).

### Catkin Packages (ROS Catkin)

These use standard catkin:
- `online_fgo_ros1`
- `kimera-vio-ros`
- `pose_graph_tools`

---

## Build Instructions

### Initial Setup (First Time)

1. **Build the base image** (if not already built):
   ```bash
   cd /workspaces/src/ros-cuda-dockers
   make build  # Builds ubuntu_base:noetic
   ```

2. **Open in Dev Container**:
   - Open VS Code in the `/workspaces/src` directory
   - `Remote-Containers: Reopen in Container`
   - Select `kimera-vio-ros/.devcontainer/devcontainer.json`

3. **Clone dependencies**:
   ```bash
   cd /workspaces/src
   vcs import < workspace.repos
   ```

4. **Setup workspace** (copies package.xml files, initializes catkin):
   ```bash
   ./scripts/setup_workspace.sh
   ```

5. **Build all packages** (15-30 minutes first time):
   ```bash
   ./scripts/build_workspace.sh
   ```

6. **Source the workspace**:
   ```bash
   source devel/setup.bash
   ```

That's it! All dependencies (GTSAM, OpenGV, etc.) are now built in your catkin workspace.

### Development Workflow

#### Quick Rebuild of Frequently Edited Packages

When you're actively developing, use the quick rebuild script:

```bash
cd /workspaces/src

# Make your code changes to online_fgo_core, Kimera-VIO, etc.

# Quick rebuild (2-5 minutes)
./scripts/rebuild_dev.sh

# Source workspace
source devel/setup.bash
```

This rebuilds only: `online_fgo_core`, `Kimera-VIO`, `online_fgo_ros1`, `kimera_vio_ros`

#### Manual Rebuild

```bash
# Rebuild just one package
catkin build online_fgo_core

# Rebuild without dependencies (faster if deps unchanged)
catkin build online_fgo_core --no-deps

# Rebuild and all packages that depend on it
catkin build --start-with online_fgo_core
```

#### Clean Rebuild

```bash
# Clean rebuild a specific package (use the script)
./scripts/clean_rebuild.sh online_fgo_core

# Or manually
catkin clean online_fgo_core
catkin build online_fgo_core

# Nuclear option - clean everything
catkin clean -y
./scripts/build_workspace.sh
```

---

## Dependency Management

### Adding New System Dependencies (Rare)

If you need a new apt package or system library:
1. Add it to `kimera-vio-ros/.devcontainer/Dockerfile`
2. Rebuild the container: `Remote-Containers: Rebuild Container`

Example: `libfoo-dev`, `python3-bar`, etc.

### Adding New ROS/Catkin Dependencies

If you need a new ROS package or C++ library:
1. Add repository to `workspace.repos` (recommended)
2. Run `vcs import < workspace.repos`
3. Run `catkin build` to build it

Or manually:
```bash
cd /workspaces/src
git clone <repo-url>
catkin build <package-name>
```

### Updating GTSAM or Heavy Dependencies

Since GTSAM and other heavy dependencies are in the catkin workspace:
1. Navigate to the package: `cd /workspaces/src/gtsam`
2. Update: `git checkout <new-version>` or `git pull`
3. Clean and rebuild: `catkin clean gtsam && catkin build gtsam`
4. Rebuild dependent packages: `catkin build --start-with gtsam`

This is much faster than rebuilding the entire Docker image!

---

## Important Notes

### CUDA/GPU Support: NOT INCLUDED

**Where skipped**:
- CUDA runtime and development libraries are NOT installed
- No NVIDIA-specific optimizations
- GPU-accelerated features (if any) will not work

**Why**: You don't have an NVIDIA GPU in your dev environment.

**If you need CUDA later**:
- Use `ros-cuda-dockers/Dockerfile.ros2` as reference for CUDA 12+ setup
- Add CUDA runtime to base image
- Enable CUDA flags in CMake builds

### ROS2 Support: NOT INCLUDED

**Where skipped**:
- `online_fgo_ros2/` package is ignored in all builds
- ROS2 dependencies not installed
- Bridging between ROS1/ROS2 not configured

**Why**: This workspace targets ROS1 (Noetic) only. ROS2 is a separate branch concern.

**How ignored**:
- `online_fgo_ros2/` has `CATKIN_IGNORE` file (or should have one)
- Build scripts explicitly exclude it
- Not listed in `workspace.repos`

### gnssFGO Package: IGNORED

**Where**: `/workspaces/src/gnssFGO/` directory

**Why**: This is a separate repository with its own build system. It's not part of this integrated workspace.

**How ignored**:
- Has `CATKIN_IGNORE` file at root
- Not referenced in build scripts
- Catkin will skip it automatically

---

## Troubleshooting

### "Package 'online_fgo_core' not found"

Make sure the CMake package is built and installed:
```bash
catkin build online_fgo_core
source devel/setup.bash
```

### "GTSAM not found" during build

GTSAM should be in the base image. Verify:
```bash
ls /usr/local/lib/libgtsam*
ls /usr/local/include/gtsam/
```

If missing, rebuild the container.

### Catkin can't find dependencies

Make sure you've sourced the workspace:
```bash
source /opt/ros/noetic/setup.bash
source /workspaces/src/devel/setup.bash
```

### Package builds but can't be found by dependent packages

For CMake packages (`online_fgo_core`, `Kimera-VIO`), check:
```bash
# Verify install happened
ls /workspaces/src/install/
ls /workspaces/src/devel/

# Check CMake config files
find /workspaces/src -name "*Config.cmake"
```

---

## File Structure Summary

```
/workspaces/src/
├── kimera-vio-ros/.devcontainer/  # Dev container config (YOU ARE HERE)
│   ├── Dockerfile                  # Base image definition
│   ├── devcontainer.json          # VS Code container config
│   ├── entrypoint.sh              # Environment setup (sources only)
│   └── DEVCONTAINER_SETUP.md      # This file
│
├── online_fgo_core/               # Core FGO library (CMake)
├── online_fgo_ros1/               # ROS1 adapter (Catkin)
├── Kimera-VIO/                    # VIO library (CMake)
├── kimera-vio-ros/                # VIO ROS wrapper (Catkin)
├── aria_common/                   # Aria utilities (CMake)
├── aria_visualization/            # Aria viz (CMake)
│
├── gnssFGO/                       # [IGNORED] Separate project
├── online_fgo_ros2/               # [IGNORED] ROS2 not supported
│
├── devel/                         # Catkin workspace overlays
├── build/                         # Build artifacts
├── install/                       # Install space
└── .catkin_tools/                 # Catkin config
```

---

## Next Steps

1. **Create `workspace.repos`** file for easy dependency cloning (see example below)
2. **Add `CATKIN_IGNORE`** to `gnssFGO/` and `online_fgo_ros2/`
3. **Verify all `package.xml`** files have correct `<build_type>` tags
4. **Create build helper scripts** for quick rebuilds during development

### Build Scripts Available

See `scripts/README.md` for full documentation. Quick reference:

```bash
# Initial setup
./scripts/setup_workspace.sh

# Build everything (first time)
./scripts/build_workspace.sh

# Quick rebuild during development
./scripts/rebuild_dev.sh

# Clean rebuild a specific package
./scripts/clean_rebuild.sh <package>
```
