# Production Point GUI

A Streamlit operator console for updating robot joint points, gripper
constants, and marker teach data without editing source files by hand.
The GUI lives under `oms_v1/gui/` so it ships with the rest of the
`oms_v1` ROS2 package.

## Production deployment (recommended)

The robot pods (`robot1`, `robot2`) ship with `streamlit` pre-installed and
bind-mount the NUC's BARNS git checkout into the container so writes go
straight to the working tree.

### Optional helper

Some BARNS deployments keep `launch-gui.sh` at the monorepo root. When it is
available, it handles cleanup-then-launch and resolves the right port + node IP
for the URL automatically:

```bash
ssh qss@192.168.200.254
cd ~/BARNS
./launch-gui.sh 1     # robot 1 GUI on http://<barns-NUC-IP>:8501
./launch-gui.sh 2     # robot 2 GUI on http://<qss-NUC-IP>:8502

./launch-gui.sh 1 --kill-only    # stop the GUI in robot 1's pod
./launch-gui.sh 1 8511           # custom port
```

The script must run from a host that has working `kubectl` against the
barns cluster (e.g. `qss@192.168.200.254`). The pod itself runs on
whichever node the Deployment's `nodeSelector` pins.

### Manual invocation

Use this path when `launch-gui.sh` is not available in the checkout:

```bash
# from a host with kubectl access (e.g. qss@192.168.200.254)
kubectl -n barns exec deploy/robot1 -- bash -c '
  source /opt/barns-robot/ros_ws/setup_robot_env.sh
  exec ros2 run oms_v1 oms_v1_gui --server.address 0.0.0.0 --server.port 8501
'
```

```bash
kubectl -n barns exec deploy/robot2 -- bash -c '
  source /opt/barns-robot/ros_ws/setup_robot_env.sh
  exec ros2 run oms_v1 oms_v1_gui --server.address 0.0.0.0 --server.port 8502
'
```

If you see `Port 8501/8502 is already in use`, an earlier launch left a
streamlit running in the pod (Ctrl-C from `kubectl exec` doesn't always
propagate). Kill it first:

```bash
kubectl -n barns exec deploy/robot1 -- pkill -f oms_v1_gui
kubectl -n barns exec deploy/robot1 -- pkill -f "streamlit run"
```

If your deployment has `launch-gui.sh`, it performs the same cleanup
automatically.

### Why `ros2 run`?

`oms_v1` is an ament_python package built with `colcon build
--symlink-install`. That puts the `oms_v1_gui` console_script under
`/opt/barns-robot/ros_ws/install/oms_v1/lib/oms_v1/oms_v1_gui` -- a path
ros2's package-executable lookup searches but plain shell `PATH` does not.
The Dockerfile additionally symlinks the script onto `/usr/local/bin/`
during the image build, so on **rebuilt images** you can just type
`oms_v1_gui ...` directly. The `ros2 run oms_v1 oms_v1_gui ...` form
always works regardless of the symlink.

### Where to run kubectl from

The qss NUC (`qss@192.168.200.254`) is the cluster control plane. All
`kubectl` commands need to run from there, even though robot 1's pod
runs on the barns NUC (`barns@192.168.200.109`). To run kubectl from the
barns NUC too, copy the kubeconfig over once:

```bash
# from qss NUC
scp ~/.kube/config barns@192.168.200.109:/tmp/kubeconfig

# from barns NUC
mkdir -p ~/.kube && mv /tmp/kubeconfig ~/.kube/config && chmod 600 ~/.kube/config
kubectl get nodes -n barns       # should now succeed
```

The kubeconfig already points at `https://192.168.200.254:6443`, which
the barns NUC can reach over the LAN, so no edits are needed.

Because both robot pods run with `hostNetwork: true`, the GUI is reachable
directly from your laptop without `kubectl port-forward`:

- Robot 1 GUI:   `http://192.168.200.109:8501`
- Robot 1 aruco: `http://192.168.200.109:8181`
- Robot 2 GUI:   `http://192.168.200.254:8502`
- Robot 2 aruco: `http://192.168.200.254:8182`

### Why writes persist

`k8s/services/robot1-deployment.yaml` and `robot2-deployment.yaml` mount:

| Pod path | NUC path |
|----------|----------|
| `/opt/barns-robot/ros_ws/src/oms_v1`        | `~/BARNS/services/robot_container/ros_ws/src/oms_v1` |
| `/opt/barns-robot/ros_ws/src/pickn_place/share` | `~/BARNS/services/robot_container/ros_ws/src/pickn_place/share` |

`BARNS_PICKN_PLACE_SHARE` is also set in the deployment so the patched
teach nodes route their source-share write through the hostPath mount.

This means clicking `Save update` in the GUI writes directly to the NUC's
git working tree -- `git status` shows the diff immediately. You do **not**
have to `kubectl cp` files out of the pod.

### Operator workflow

1. Train / capture in the GUI on the NUC's pod. Saved values land in the
   NUC's `~/BARNS/...` working tree.
2. From the NUC:
   ```bash
   cd ~/BARNS
   git status
   git diff
   git add -A
   git commit -m "robot1: update points + teach via GUI"
   git push
   ```
3. From wherever you build images (e.g. your laptop):
   ```bash
   git pull
   ./barns-deploy.sh   # option 2 -> b/c (just that robot) -> N (use cache) -> Y push
   ```
4. K8s rolls the new image; future pod restarts retain the values because
   they are baked into the image **and** the hostPath copy is identical.

### Prerequisites

- The BARNS repo must already be cloned on each NUC at `~/BARNS` for the
  matching user (`barns@192.168.200.109` / `qss@192.168.200.254`). If the
  repo is missing the pod will fail to start (`type: Directory` on the
  hostPath volumes is intentional -- silent fallback is worse).
- Files written by the GUI end up owned by `root` because the container
  runs as root. Git can still read them, so `git diff` / `git commit` work
  without sudo. To manually edit a saved file from a shell you would need
  `sudo`.

## Manual / development run

If you don't want to use the K8s pod (e.g. running the GUI on a developer
laptop with ROS2 sourced), the package can also be run directly:

```bash
cd services/robot_container/ros_ws/src/oms_v1
pip install -e ".[gui]"
streamlit run oms_v1/gui/streamlit_app.py
```

After `colcon build` has run and the workspace is sourced the console
script also works:

```bash
oms_v1_gui
```

The GUI does not need to run on the same host as the dobot stack. When
running on a host with ROS2 sourced and `oms_v1.manipulate_node`
importable, the GUI uses the direct `run_skill` path; otherwise it falls
back to the existing RabbitMQ `execute_action` API. Note that the
RabbitMQ fallback only exposes the high-level toggles (init / drag /
open-close gripper) -- live joint readout, numeric gripper apply, and
marker training all require the direct backend.

## Top-level menu

| Option | Purpose |
|--------|---------|
| Update points | Capture and persist static joint poses / gripper widths used by sequence functions |
| Update marker training | Run `tool_mount_teach` and `machine_mount_teach` from the GUI and save into the deployment share folders |
| Exit | Closes the operator session (close the browser tab to fully stop) |

The persistent sidebar (visible at all times) provides:

- Backend status (direct ROS2 / RabbitMQ / offline)
- Initialize robot (mirrors the six-step init in `robot1-startup.sh`)
- Toggle drag mode
- Open / Close gripper, numeric position slider + Apply
- Live joint and pose readout

## Update points workflow

1. Pick a sequence function (only catalogued ones in
   `oms_v1/gui/point_catalog.py` are shown so non-teachable internals stay
   hidden).
2. Pick variant axes (e.g. paper-cup size, espresso port, slush dispenser).
   Options are read from `params.py` at runtime so adding a new entry to
   `GRAB_PAPER_CUP_PARAMS` automatically appears in the GUI.
3. For each editable point:
   - Old value, new captured value, target params key, and an optional
     reference image are shown side-by-side.
   - `Modify` -> drag to position -> `Update` to capture
     `current_angles` (joint poses) or the slider value (gripper widths)
   - Confirm via a checkbox before `Save update`.
4. After all points are saved or skipped, optionally test-play the
   sequence with the chosen variant.

`params.py` updates are AST-located, atomic, and validated:

- A copy is taken to `oms_v1/params.py.bak/<utc_iso>.py` first.
- The new literal is spliced into the existing source range so comments and
  whitespace are preserved.
- The file is written via `tempfile + os.replace`.
- `compile()` validates the result; failure restores the backup.

Only Python literals (numbers, strings, tuples, lists, dicts of those) are
accepted. The editor refuses anything else.

## Update marker training workflow

The GUI reuses the camera (`orbbec_camera gemini_330_series.launch.py`)
and perception node (`pickn_place aruco_perception`) that the robot pod
already runs via `robot{1,2}-startup.sh`. The marker training screen
embeds the live MJPEG stream from the existing `perception_streamer`
(port 8181 / 8182) directly in the page, with aruco markers labeled.

RViz is **not** offered. The robot pods are headless K8s containers
with no X display, so `rviz2` fails with
`Couldn't open X display :99`. The embedded MJPEG stream is the
intended visual feedback for marker training -- it's the same source
the dashboard uses.

### Tool teach

Pick a tool marker, click `Train`. The GUI feeds the marker name into
`ros2 run pickn_place tool_mount_teach`'s prompt, streams its log, and
parses the `Tool offset save target: ...` lines emitted by the patched
node so the operator can see exactly which YAML files were written.

### Machine teach

Pick a machine marker and `Sample marker`. Once averaging finishes,
choose a point, fill in the approach + mount TCP frames (default `Link6`),
and `Save approach + mount`. The GUI walks the same prompt sequence
(`prompt_for_frame` -> `prompt_and_sample`) the original CLI flow uses.

When done, `Finalize and save YAML` ends the point loop and produces
the saved-to lines.

## Save paths

Both teach nodes were patched to resolve their save paths via:

1. `BARNS_PICKN_PLACE_SHARE` environment variable (if it points to an
   existing directory).
2. A walk up from `__file__` looking for
   `services/robot_container/ros_ws/src/pickn_place/share` under the BARNS
   repo root.
3. Fallback to the legacy `~/barns_ws/src/pickn_place/share` only if none
   of the above resolve.

In addition, the install share directory (`get_package_share_directory(
"pickn_place")`) is always written so a running stack picks up changes
immediately.

A `.bak/<utc_iso>.yaml` snapshot is taken next to each YAML before
overwriting.

## Trace logging

The GUI mirrors the existing `[SCOPE] message` style used by
`oms_v1.sequences.espresso` so a single `grep` over the robot log catches
both sequence and GUI activity. Scopes used:

| Scope | Source |
|-------|--------|
| `GUI` | top-level UI events |
| `GUI-BRIDGE` | RobotBridge / ROS2 calls |
| `GUI-PARAMS` | params.py read/write |
| `GUI-CAT` | point catalog lookups |
| `GUI-REG` | sequence registry |
| `GUI-PROC[<name>]` | camera / perception subprocesses |
| `GUI-TEACH[<label>]` | teach subprocess I/O |
| `GUI-MARKER` | marker catalog discovery |

A bounded ring buffer (~2000 lines) is kept in process so the bottom
panel of the GUI shows the same lines without re-tailing log files.

## Assumptions and limitations

- Only the catalogued functions in `point_catalog.py` are exposed for
  point editing; broaden the catalog as new sequences become teachable.
- Direct ROS2 backend (rclpy + `oms_v1.manipulate_node`) is required for
  numeric gripper application and live joint/pose readout. The RabbitMQ
  fallback only exposes high-level toggles.
- Test-play runs the sequence in a worker thread; if the underlying ROS2
  stack is wedged the operator must use the sidebar `Initialize robot`
  before retrying.
- Existing teach files at the legacy `~/barns_ws/...` path are still
  written if the BARNS repo source share cannot be located -- the
  fallback is logged so the warning is visible in the GUI trace.
- `Discard last save` on the tool-teach screen requires manually picking
  a backup from the YAML's `.bak/` neighbour folder; this is intentional
  to avoid restoring a stale teach mid-shift.

## Files

| Status | Path |
|--------|------|
| added | `services/robot_container/ros_ws/src/oms_v1/oms_v1/gui/__init__.py` |
| added | `services/robot_container/ros_ws/src/oms_v1/oms_v1/gui/trace.py` |
| added | `services/robot_container/ros_ws/src/oms_v1/oms_v1/gui/robot_bridge.py` |
| added | `services/robot_container/ros_ws/src/oms_v1/oms_v1/gui/sequence_registry.py` |
| added | `services/robot_container/ros_ws/src/oms_v1/oms_v1/gui/point_catalog.py` |
| added | `services/robot_container/ros_ws/src/oms_v1/oms_v1/gui/params_editor.py` |
| added | `services/robot_container/ros_ws/src/oms_v1/oms_v1/gui/marker_catalog.py` |
| added | `services/robot_container/ros_ws/src/oms_v1/oms_v1/gui/process_manager.py` |
| added | `services/robot_container/ros_ws/src/oms_v1/oms_v1/gui/teach_runner.py` |
| added | `services/robot_container/ros_ws/src/oms_v1/oms_v1/gui/streamlit_app.py` |
| added | `services/robot_container/ros_ws/src/oms_v1/oms_v1/gui/assets/README.md` |
| added | `services/robot_container/ros_ws/src/oms_v1/docs/production_point_gui.md` |
| modified | `services/robot_container/ros_ws/src/oms_v1/setup.py` (added `oms_v1_gui` console script + `[gui]` extras) |
| modified | `services/robot_container/ros_ws/src/pickn_place/pickn_place/tool_mount_teach.py` (save-path resolver + backup) |
| modified | `services/robot_container/ros_ws/src/pickn_place/pickn_place/machine_mount_teach.py` (save-path resolver + backup) |
