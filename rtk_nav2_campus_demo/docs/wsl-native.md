# WSL native RTK/Nav2 demo

This path runs the current Fortress RTK/Nav2 campus demo directly on Ubuntu 22.04 WSL. It uses the existing `rtk_nav2_ws`, the root repository's `ros2_bridge`, and the campus model under `gazebo_campus/generated/models`. It does not build or start a Docker container.

## Prerequisites

- ROS 2 Humble, Nav2, Gazebo Fortress, `ros_gz_sim`, `ros_gz_bridge`, `robot_localization`, `depth_image_proc`, `turtlebot3_description`, and `xacro` installed through apt.
- A working WSLg X display. The Gazebo GUI must run because this demo's depth camera requires rendering.
- `mesa-utils` (`glxinfo`) for the OpenGL compatibility check.
- Node.js and npm for the optional web interface.

The WSL entrypoint uses `/usr/bin/python3` and puts Ubuntu's Python packages before locally installed packages. This avoids the host's conda Python and an incompatible `/usr/local` setuptools/packaging combination. It also exposes the package's existing geometry scripts to its ROS nodes.
The entrypoint defaults to automatic renderer selection. It prefers the NVIDIA adapter through `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`, checks the actual hardware OpenGL context for `GL_ARB_copy_image`, and falls back to Mesa software rendering when that extension is unavailable. The system Mesa 23.2.1 exposes OpenGL 4.2 on this RTX 4070 Laptop, but lacks `GL_ARB_copy_image`; Ogre Next 2.2.5 aborts in `GL3PlusTextureGpu::copyTo` on that path. The optional private Mesa 25.0.7 installation described below exposes OpenGL 4.6 with that extension and runs the demo using the GPU. Software rendering remains available and uses CPU resources.

Use `--render-software` to select software rendering directly. `--render-hardware` requires a compatible hardware context and reports an error before launch otherwise. The flags control `LIBGL_ALWAYS_SOFTWARE`, including when that variable is inherited from the shell. Hardware and selected renderer details are saved as `logs/render_hardware.log` and `logs/render_selected.log`. Override `MESA_D3D12_DEFAULT_ADAPTER_NAME` to select a different WSLg adapter.

## Start

From the repository root, use two terminals:

```bash
bash rtk_nav2_campus_demo/scripts/rtk-nav2/wsl-start.sh
bash rtk_nav2_campus_demo/scripts/rtk-nav2/wsl-frontend.sh
```

The first command builds the ROS workspace on first use, starts Gazebo and Nav2, waits for the RTK, camera, and costmap signals, then remains in the foreground. The second command starts the ROS bridge and Vite. Open `http://localhost:5174` after it prints the URL. Set `FRONTEND_PORT` to use another port; a busy port is reported as an error. `--rviz` enables RViz; `--rebuild` recompiles ROS packages after source changes.

To stop both processes from another terminal:

```bash
bash rtk_nav2_campus_demo/scripts/rtk-nav2/wsl-stop.sh
```

Logs are written under `rtk_nav2_campus_demo/artifacts/rtk-nav2/wsl/logs/`. The original Docker entrypoints remain available for the image-based setup.

## Private Mesa 25.0.7 installation

This host has Mesa 25.0.7 installed in `~/.local/share/outdoornav/mesa-25.0.7`. It consists of Ubuntu 22.04 packages extracted from the [kisak-mesa stable archive](https://launchpad.net/~kisak/+archive/ubuntu/turtle), with the archive signature, package index checksum, and each package's SHA256 verified. `packages.txt` records package versions and hashes; the `downloads` directory retains the packages and signed metadata.

The system Mesa packages, apt sources, shell startup files, WSL kernel, and Windows drivers are not changed by this private installation. `wsl-start.sh` sources `wsl-mesa-env.sh`, which selects the private libraries only for the launched processes. It explicitly chooses `GALLIUM_DRIVER=d3d12` for GPU rendering because the new generic DRI loader otherwise selects software rendering on this host. Software mode explicitly chooses `llvmpipe`.

No WSL restart is required. The usual `wsl-start.sh` command automatically uses the private installation when it is present. Set `WSL_MESA_PREFIX` to another installation directory to override it, or to `system` to bypass it.

To return the demo to the original system libraries and software rendering, stop the demo and then run this in a fresh terminal:

```bash
bash rtk_nav2_campus_demo/scripts/rtk-nav2/wsl-stop.sh
WSL_MESA_PREFIX=system bash rtk_nav2_campus_demo/scripts/rtk-nav2/wsl-start.sh --render-software
```

To check the private GPU renderer without changing the current terminal's environment:

```bash
bash -c 'source rtk_nav2_campus_demo/scripts/rtk-nav2/wsl-mesa-env.sh; MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA LIBGL_ALWAYS_SOFTWARE=0 glxinfo -B'
```

## WSL validation (2026-10-14)

With private Mesa 25.0.7, the forced hardware entrypoint selected `D3D12 (NVIDIA GeForce RTX 4070 Laptop GPU)` / OpenGL 4.6 and passed full readiness in 3.6 seconds. Gazebo's own Ogre2 log confirmed the same GPU and Mesa version. Depth images, point clouds, lidar, and Nav2 remained active through the subsequent navigation check, without the previous Ogre2 abort. A 1 m target test completed in 5 seconds, with 0.2226 m reported distance to target at completion and physical movement recorded by Gazebo. See `logs/navigation_smoke_mesa25.log` and `logs/render_selected.log`. A separate software-context check still provided OpenGL 4.5 through llvmpipe; a plain terminal continued using system Mesa 23.2.1 / OpenGL 4.2. A complete campus patrol has not been tested on this host.

The earlier system-Mesa validation follows for reference:

On the RTX 4070 Laptop / WSL2 host, the updated automatic entrypoint selected llvmpipe and passed readiness in 24 seconds: the simulator, map-to-robot TF, depth images, point clouds, lidar, and both Nav2 costmaps were available. The forced hardware mode exited before launch with the missing-extension diagnostic. The web UI and its `/api/status` proxy returned live robot poses; a Chromium browser check reported no page errors. A 1 m navigation smoke test went through `queued → running → completed` in 7 seconds, with 0.248 m reported distance to the target at completion. The trajectory recorder also confirmed physical movement in Gazebo. This validates startup and a short navigation task; a complete campus patrol has not been tested on this host.

The smoke log is `artifacts/rtk-nav2/wsl/logs/navigation_smoke.log`, and the browser screenshot is `artifacts/rtk-nav2/wsl/web-ui.png`. To repeat the navigation check with the stack and web bridge running (this moves the simulated robot):

```bash
conda run --no-capture-output -n torch python rtk_nav2_campus_demo/scripts/rtk-nav2/frontend-smoke.py --len 1 --timeout 90
```
