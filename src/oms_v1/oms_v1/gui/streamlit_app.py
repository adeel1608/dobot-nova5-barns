"""
gui/streamlit_app.py

Production-point and marker-training GUI entrypoint.

Run with:

    cd services/robot_container/ros_ws/src/oms_v1
    streamlit run oms_v1/gui/streamlit_app.py

Or, after ``colcon build`` and sourcing the workspace:

    oms_v1_gui

The app deliberately keeps a small, predictable layout:

    - Persistent sidebar (always visible) carries Initialize/Drag/Gripper
    - Main panel routes between the three top-level menu options
      (Update points, Update marker training, Exit) and their sub-flows
    - A trailing trace panel shows the most recent ``[GUI*]`` log lines
"""

from __future__ import annotations

import os
import queue
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

try:
    import streamlit as st
except ImportError as exc:  # pragma: no cover - clearer message than streamlit's own
    raise SystemExit(
        "streamlit is not installed in this environment. "
        "Install with `pip install streamlit` and re-run."
    ) from exc

_PKG_ROOT = Path(__file__).resolve().parent.parent
if str(_PKG_ROOT.parent) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT.parent))

from oms_v1.gui import (  # noqa: E402
    marker_catalog,
    params_editor,
    point_catalog,
    process_manager,
    robot_bridge,
    sequence_registry,
    teach_runner,
    trace,
)


PAGE_TITLE = "BARNS Production Point GUI"


def _safe_rerun() -> None:
    """Streamlit renamed `experimental_rerun` -> `rerun` around 1.27 and
    removed the old name entirely in 1.30+. Use whichever the installed
    streamlit exposes so the GUI works on both old and new releases."""
    rerun = getattr(st, "rerun", None) or getattr(st, "experimental_rerun", None)
    if rerun is None:
        return
    rerun()


# ---------------------------------------------------------------------------
# session state
# ---------------------------------------------------------------------------


def _ensure_state() -> None:
    defaults = {
        "menu": "main",
        "selected_function": None,
        "function_choice": {},
        "current_point_idx": 0,
        "captured_value": None,
        "pending_save": None,
        "test_play_running": False,
        "test_play_result": None,
        "test_play_queue": queue.Queue(),
        # Tool teach
        "tool_session": None,
        "tool_session_marker": None,
        # Machine teach: phase-based state machine.
        # Phases:
        #   "idle"             -- no session yet
        #   "marker_ready"     -- subprocess up, marker averaged, ready for
        #                         a point name
        #   "approach_pending" -- point name sent, operator should drag the
        #                         robot to the approach pose, then click
        #                         "Capture approach"
        #   "mount_pending"    -- approach captured, operator should drag to
        #                         mount pose, then click "Capture mount"
        #   "point_done"       -- both poses captured for this point;
        #                         operator can train another or finalize
        "machine_session": None,
        "machine_phase": "idle",
        "machine_marker": None,
        "machine_current_point": None,
        "machine_approach_tcp": "Link6",
        "machine_mount_tcp": "Link6",
        "machine_captured_points": [],
        "trace_paused": False,
        "gripper_target": 0,
        "live_state": {},
        "robot_control_busy": False,
    }
    for key, val in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = val


# ---------------------------------------------------------------------------
# helpers shared between pages
# ---------------------------------------------------------------------------


def _bridge() -> robot_bridge.RobotBridge:
    return robot_bridge.get_bridge()


def _proc_mgr() -> process_manager.ProcessManager:
    return process_manager.get_process_manager()


def _format_value_short(value: Any, max_len: int = 220) -> str:
    if value is None:
        return "<not set>"
    text = repr(value)
    if len(text) > max_len:
        text = text[: max_len - 3] + "..."
    return text


def _go_to_main() -> None:
    st.session_state["menu"] = "main"
    st.session_state["selected_function"] = None
    st.session_state["function_choice"] = {}
    st.session_state["current_point_idx"] = 0
    st.session_state["captured_value"] = None
    st.session_state["pending_save"] = None


def _run_sidebar_robot_action(label: str, fn) -> None:
    if st.session_state.get("robot_control_busy"):
        st.warning("A robot control action is already running.")
        trace.gui_log("GUI", f"sidebar: {label} rejected busy")
        return

    st.session_state["robot_control_busy"] = True
    trace.gui_log("GUI", f"sidebar: {label} start")
    try:
        ok, msg = fn()
        trace.gui_log(
            "GUI",
            f"sidebar: {label} {'success' if ok else 'failure'} msg={msg}",
        )
        (st.success if ok else st.error)(msg)
    except Exception as exc:
        trace.gui_log_error("GUI", f"sidebar: {label} raised", exc)
        st.error(f"{label} failed: {type(exc).__name__}: {exc}")
    finally:
        st.session_state["robot_control_busy"] = False


# ---------------------------------------------------------------------------
# sidebar
# ---------------------------------------------------------------------------


def _render_sidebar() -> None:
    bridge = _bridge()
    backend = bridge.backend()
    robot_busy = bool(st.session_state.get("robot_control_busy"))

    with st.sidebar:
        st.header("Robot controls")
        backend_label = {
            "direct": "Direct ROS2",
            "rabbitmq": "RabbitMQ fallback",
            "unavailable": "OFFLINE",
        }.get(backend, backend)
        if backend == "direct":
            st.success(f"Backend: {backend_label}")
        elif backend == "rabbitmq":
            st.info(f"Backend: {backend_label}")
        else:
            st.error(
                "Backend: OFFLINE -- "
                "neither rclpy nor RabbitMQ client could be imported"
            )

        if st.button("Re-detect backend", key="sidebar_redetect", disabled=robot_busy):
            bridge.reset_direct_backend()
            _safe_rerun()

        if st.button(
            "Initialize robot",
            key="sidebar_init",
            disabled=(robot_busy or backend != "direct"),
        ):
            _run_sidebar_robot_action("Initialize robot", bridge.initialize_robot)

        c_enable, c_disable = st.columns(2)
        if c_enable.button(
            "Enable robot",
            key="sidebar_enable",
            disabled=(robot_busy or backend != "direct"),
        ):
            _run_sidebar_robot_action("Enable robot", bridge.enable_robot)
        if c_disable.button(
            "Disable robot",
            key="sidebar_disable",
            disabled=(robot_busy or backend != "direct"),
        ):
            _run_sidebar_robot_action("Disable robot", bridge.disable_robot)

        if st.button(
            "Toggle drag mode",
            key="sidebar_drag",
            disabled=(robot_busy or backend == "unavailable"),
        ):
            _run_sidebar_robot_action("Toggle drag mode", bridge.toggle_drag)

        st.markdown("---")
        st.subheader("Gripper")
        c1, c2 = st.columns(2)
        if c1.button(
            "Open",
            key="grip_open",
            disabled=(robot_busy or backend == "unavailable"),
        ):
            _run_sidebar_robot_action("Open gripper", bridge.open_gripper)
        if c2.button(
            "Close",
            key="grip_close",
            disabled=(robot_busy or backend == "unavailable"),
        ):
            _run_sidebar_robot_action("Close gripper", bridge.close_gripper)

        st.session_state["gripper_target"] = st.slider(
            "Gripper position (0=open, 255=full)",
            min_value=0,
            max_value=255,
            value=int(st.session_state.get("gripper_target", 0)),
            step=1,
            key="grip_slider",
        )
        if st.button(
            "Apply gripper position",
            key="grip_apply",
            disabled=(robot_busy or backend != "direct"),
        ):
            value = int(st.session_state["gripper_target"])
            _run_sidebar_robot_action(
                f"Apply gripper position={value}",
                lambda: bridge.set_gripper_position(value),
            )

        st.markdown("---")
        st.subheader("Live state")
        if st.button("Refresh live state", key="sidebar_refresh"):
            bridge.current_angles()
            bridge.current_pose()
        last = bridge.last_state()
        joints = last.get("joints")
        pose = last.get("pose")
        if joints:
            st.caption("Joints (deg)")
            st.code(", ".join(f"{v:.3f}" for v in joints))
        else:
            st.caption("Joints: (not yet read)")
        if pose:
            st.caption("Pose (mm/deg)")
            st.code(", ".join(f"{v:.3f}" for v in pose))
        else:
            st.caption("Pose: (not yet read)")

        if st.button("Return to main menu", key="sidebar_main"):
            _go_to_main()
            _safe_rerun()


# ---------------------------------------------------------------------------
# main menu
# ---------------------------------------------------------------------------


def _render_main_menu() -> None:
    st.header("Main menu")
    st.write(
        "Choose what to update. Use the sidebar at any time to initialize the "
        "robot, toggle drag mode, or move the gripper."
    )

    c1, c2, c3 = st.columns(3)
    if c1.button("Update points", key="menu_update_points"):
        trace.gui_log("GUI", "menu: Update points")
        st.session_state["menu"] = "update_points"
        _safe_rerun()
    if c2.button("Update marker training", key="menu_update_markers"):
        trace.gui_log("GUI", "menu: Update marker training")
        st.session_state["menu"] = "marker_training"
        _safe_rerun()
    if c3.button("Exit", key="menu_exit"):
        trace.gui_log("GUI", "menu: Exit pressed")
        st.warning(
            "To exit, close the browser tab and stop the streamlit server "
            "with Ctrl-C in the terminal."
        )


# ---------------------------------------------------------------------------
# update points flow
# ---------------------------------------------------------------------------


def _list_function_options() -> List[str]:
    return sequence_registry.public_sequence_names()


def _drain_test_play_queue() -> None:
    result_queue = st.session_state.get("test_play_queue")
    if result_queue is None:
        result_queue = queue.Queue()
        st.session_state["test_play_queue"] = result_queue
    while True:
        try:
            result = result_queue.get_nowait()
        except queue.Empty:
            break
        st.session_state["test_play_result"] = result
        st.session_state["test_play_running"] = False


def _render_update_points_index() -> None:
    st.header("Update points")
    st.write(
        "Pick a registered sequence function. Functions without point metadata "
        "can still be selected for an optional test run."
    )

    options = _list_function_options()
    errors = sequence_registry.load_errors()
    if errors:
        with st.expander("Sequence import warnings"):
            for mod, err in errors:
                st.warning(f"{mod}: {err}")

    if not options:
        st.error("No sequence functions available in this environment.")
        if st.button("Back", key="up_no_options_back"):
            _go_to_main()
            _safe_rerun()
        return

    selection = st.selectbox(
        "Function", options, index=0, key="update_points_function_select"
    )

    meta = point_catalog.metadata(selection)
    if meta is not None:
        st.info(f"Summary: {meta.summary}")
    else:
        st.info("Registered sequence; no editable point metadata is catalogued yet.")

    c1, c2 = st.columns(2)
    if c1.button("Confirm", key="up_confirm"):
        trace.gui_log("GUI", f"update_points: selected function={selection}")
        st.session_state["selected_function"] = selection
        st.session_state["function_choice"] = {}
        st.session_state["current_point_idx"] = 0
        st.session_state["menu"] = "update_points_variant"
        _safe_rerun()
    if c2.button("Back", key="up_back"):
        _go_to_main()
        _safe_rerun()


def _render_update_points_variant() -> None:
    fn_name = st.session_state.get("selected_function")
    if not fn_name:
        st.session_state["menu"] = "update_points"
        _safe_rerun()
        return

    meta = point_catalog.metadata(fn_name)
    st.header(f"{fn_name}")
    if meta is not None:
        st.caption(meta.summary)

    if meta is None:
        st.info("No editable points are registered for this sequence.")
        st.session_state["function_choice"] = {}
        st.session_state["menu"] = "update_points_test_play"
        _safe_rerun()
        return

    if not meta.variant_axes:
        st.session_state["function_choice"] = {}
        st.session_state["menu"] = "update_points_edit"
        _safe_rerun()
        return

    st.subheader("Choose variant")
    choice: Dict[str, Any] = dict(st.session_state.get("function_choice", {}))
    for axis_id, axis_label in meta.variant_axes:
        options = point_catalog.variant_options(fn_name, axis_id)
        if not options:
            st.error(
                f"No options found for axis {axis_id!r} -- "
                f"params.py may not define the corresponding dictionary."
            )
            continue
        default = choice.get(axis_id, options[0])
        choice[axis_id] = st.selectbox(
            axis_label,
            options,
            index=options.index(default) if default in options else 0,
            key=f"variant_{fn_name}_{axis_id}",
        )

    c1, c2 = st.columns(2)
    if c1.button("Confirm variant", key="variant_confirm"):
        trace.gui_log(
            "GUI", f"update_points: variant chosen fn={fn_name} choice={choice}"
        )
        st.session_state["function_choice"] = choice
        st.session_state["current_point_idx"] = 0
        st.session_state["captured_value"] = None
        st.session_state["pending_save"] = None
        st.session_state["menu"] = "update_points_edit"
        _safe_rerun()
    if c2.button("Back", key="variant_back"):
        st.session_state["menu"] = "update_points"
        _safe_rerun()


def _render_update_points_edit() -> None:
    fn_name = st.session_state.get("selected_function")
    choice = dict(st.session_state.get("function_choice", {}))
    if not fn_name:
        st.session_state["menu"] = "update_points"
        _safe_rerun()
        return

    points = point_catalog.points_for(fn_name, choice)
    if not points:
        st.warning("No editable points are registered for this selection.")
        if st.button("Back", key="edit_empty_back"):
            st.session_state["menu"] = "update_points_variant"
            _safe_rerun()
        return

    idx = int(st.session_state.get("current_point_idx", 0))
    if idx >= len(points):
        st.session_state["menu"] = "update_points_test_play"
        _safe_rerun()
        return

    point = points[idx]
    st.header(f"{fn_name}: point {idx + 1}/{len(points)}")
    st.subheader(point.name)
    st.caption(point.description)

    target = params_editor.resolve_target(point.params_path[0], point.params_path[1:])
    current_value = target.current_value if target else None
    target_key_path = params_editor.format_key_path(
        point.params_path[0], point.params_path[1:]
    )
    st.markdown(f"**Target key**: `{target_key_path}`")
    st.markdown(f"**Kind**: `{point.kind}`")
    st.markdown(f"**Current saved value**: `{_format_value_short(current_value)}`")

    captured = st.session_state.get("captured_value")
    if captured is not None:
        st.markdown(f"**Captured value**: `{_format_value_short(captured)}`")

    bridge = _bridge()
    last = bridge.last_state()
    if last.get("joints"):
        st.caption("Live joints: " + ", ".join(f"{v:.3f}" for v in last["joints"]))
    if last.get("pose"):
        st.caption("Live pose:   " + ", ".join(f"{v:.3f}" for v in last["pose"]))

    asset = _PKG_ROOT / "gui" / "assets" / f"{target_key_path}.png"
    if asset.exists():
        st.image(str(asset), caption=str(asset.name))

    cols = st.columns(4)
    if cols[0].button("Modify", key=f"pt_modify_{idx}"):
        trace.gui_log(
            "GUI", f"update_points: Modify pressed key={target_key_path}"
        )
        st.session_state["menu"] = "update_points_capture"
        _safe_rerun()
    if cols[1].button("Skip", key=f"pt_skip_{idx}"):
        trace.gui_log(
            "GUI", f"update_points: Skip pressed key={target_key_path}"
        )
        st.session_state["captured_value"] = None
        st.session_state["pending_save"] = None
        st.session_state["current_point_idx"] = idx + 1
        _safe_rerun()
    if cols[2].button("Back", key=f"pt_back_{idx}"):
        st.session_state["menu"] = "update_points_variant"
        _safe_rerun()
    if cols[3].button("Finish (skip rest)", key=f"pt_finish_{idx}"):
        st.session_state["menu"] = "update_points_test_play"
        _safe_rerun()


def _render_update_points_capture() -> None:
    fn_name = st.session_state.get("selected_function")
    choice = dict(st.session_state.get("function_choice", {}))
    if not fn_name:
        st.session_state["menu"] = "update_points"
        _safe_rerun()
        return
    points = point_catalog.points_for(fn_name, choice)
    idx = int(st.session_state.get("current_point_idx", 0))
    if idx >= len(points):
        st.session_state["menu"] = "update_points_edit"
        _safe_rerun()
        return

    point = points[idx]
    bridge = _bridge()
    target_key_path = params_editor.format_key_path(
        point.params_path[0], point.params_path[1:]
    )

    st.header(f"Capture for {target_key_path}")
    st.write(
        "1. Use the sidebar to toggle drag mode if needed.\n"
        "2. Move the robot to the desired pose.\n"
        "3. Click Update to capture the current value."
    )

    captured = st.session_state.get("captured_value")

    bridge.current_angles()
    bridge.current_pose()
    last = bridge.last_state()
    if last.get("joints"):
        st.caption("Live joints: " + ", ".join(f"{v:.3f}" for v in last["joints"]))
    if last.get("pose"):
        st.caption("Live pose:   " + ", ".join(f"{v:.3f}" for v in last["pose"]))

    cols = st.columns(4)
    if cols[0].button("Update (capture now)", key=f"cap_update_{idx}"):
        if point.kind == "joint":
            angles = bridge.current_angles()
            if angles is None or len(angles) < 6:
                st.error(
                    "Could not read current joint angles. "
                    "Direct ROS2 backend is required."
                )
            else:
                rounded = tuple(round(float(a), 6) for a in angles[:6])
                st.session_state["captured_value"] = rounded
                trace.gui_log(
                    "GUI",
                    f"update_points: captured joints={rounded} "
                    f"key={target_key_path}",
                )
        elif point.kind == "gripper":
            value = int(st.session_state.get("gripper_target", 0))
            st.session_state["captured_value"] = value
            trace.gui_log(
                "GUI",
                f"update_points: captured gripper={value} key={target_key_path}",
            )
        _safe_rerun()
    if cols[1].button("Recapture", key=f"cap_recap_{idx}"):
        st.session_state["captured_value"] = None
        _safe_rerun()
    if cols[2].button("Discard", key=f"cap_discard_{idx}"):
        trace.gui_log(
            "GUI", f"update_points: discard capture key={target_key_path}"
        )
        st.session_state["captured_value"] = None
        st.session_state["menu"] = "update_points_edit"
        _safe_rerun()
    if cols[3].button("Back", key=f"cap_back_{idx}"):
        st.session_state["menu"] = "update_points_edit"
        _safe_rerun()

    if captured is not None:
        st.markdown("---")
        st.subheader("Confirm save")
        target = params_editor.resolve_target(
            point.params_path[0], point.params_path[1:]
        )
        current_value = target.current_value if target else None
        bak_dir = params_editor.backup_dir()
        st.markdown(f"**Target file**: `{params_editor.default_params_path()}`")
        st.markdown(f"**Target key**: `{target_key_path}`")
        st.markdown(f"**Current value**: `{_format_value_short(current_value)}`")
        st.markdown(f"**New value**: `{_format_value_short(captured)}`")
        st.markdown(f"**Backups will be written under**: `{bak_dir}`")
        confirm = st.checkbox(
            "I have reviewed the diff above",
            key=f"cap_confirm_{idx}",
        )
        if st.button(
            "Save update",
            key=f"cap_save_{idx}",
            disabled=not confirm,
        ):
            result = params_editor.update_value(
                point.params_path[0], point.params_path[1:], captured
            )
            if result.success:
                st.success(result.message)
                trace.gui_log(
                    "GUI", f"update_points: saved key={target_key_path}"
                )
                st.session_state["captured_value"] = None
                st.session_state["current_point_idx"] = idx + 1
                st.session_state["menu"] = "update_points_edit"
                _safe_rerun()
            else:
                st.error(f"save failed: {result.message}")
                trace.gui_log(
                    "GUI", f"update_points: SAVE FAILED key={target_key_path} "
                    f"msg={result.message}"
                )


def _render_update_points_test_play() -> None:
    _drain_test_play_queue()
    fn_name = st.session_state.get("selected_function")
    choice = dict(st.session_state.get("function_choice", {}))
    st.header("Test play")
    st.write(
        f"Optionally test-run **{fn_name}** with the chosen variant before "
        "moving on. The function runs in a worker thread so the GUI stays "
        "responsive; logs stream into the trace panel."
    )
    if st.session_state.get("test_play_running"):
        st.info("Test play is currently running...")
    result = st.session_state.get("test_play_result")
    if result is not None:
        if result.get("ok"):
            st.success(f"Test play succeeded: {result.get('detail')}")
        else:
            st.error(f"Test play failed: {result.get('detail')}")

    cols = st.columns(4)
    block_reason = sequence_registry.test_play_block_reason(fn_name)
    if block_reason:
        st.warning(block_reason)
    can_run = (
        not st.session_state.get("test_play_running")
        and sequence_registry.can_test_play_sequence(fn_name)
    )
    if cols[0].button("Run test", key="tp_run", disabled=not can_run):
        sequences = sequence_registry.load_sequences()
        fn = sequences.get(fn_name)
        if fn is None:
            st.error(f"Sequence {fn_name} is not registered in this process")
        else:
            call_params = point_catalog.call_params_for(fn_name, choice)
            trace.gui_log(
                "GUI", f"test_play: running {fn_name} params={call_params}"
            )
            st.session_state["test_play_running"] = True
            st.session_state["test_play_result"] = None
            result_queue = st.session_state["test_play_queue"]

            def runner() -> None:
                try:
                    result_obj = fn(**call_params)
                    ok = result_obj not in (False, None)
                    result_queue.put({
                        "ok": ok,
                        "detail": repr(result_obj),
                    })
                except Exception as exc:
                    trace.gui_log_error("GUI", "test_play raised", exc)
                    result_queue.put({
                        "ok": False,
                        "detail": f"{type(exc).__name__}: {exc}",
                    })

            threading.Thread(
                target=runner, name="gui-test-play", daemon=True
            ).start()
            time.sleep(0.5)
            _safe_rerun()
    if cols[1].button("Skip", key="tp_skip"):
        st.session_state["menu"] = "update_points"
        st.session_state["selected_function"] = None
        _safe_rerun()
    if cols[2].button("Back to points", key="tp_back"):
        st.session_state["current_point_idx"] = 0
        st.session_state["menu"] = "update_points_edit"
        _safe_rerun()
    if cols[3].button("Main menu", key="tp_main"):
        _go_to_main()
        _safe_rerun()
    if st.session_state.get("test_play_running"):
        time.sleep(0.5)
        _safe_rerun()


# ---------------------------------------------------------------------------
# marker training flows
# ---------------------------------------------------------------------------


# Tool / machine teach session objects live in ``st.session_state`` so they
# survive Streamlit reruns even if the module's global state is ever lost
# (e.g. a watchdog-triggered reload). The TeachSession holds a `subprocess
# .Popen` which Streamlit's session_state stores by reference -- it cannot be
# pickled, but Streamlit only persists session_state in memory, so this is
# fine for the single-operator deployment.
_TOOL_SESSION_KEY = "tool_session"
_MACHINE_SESSION_KEY = "machine_session"


def _render_marker_menu() -> None:
    st.header("Update marker training")
    c1, c2, c3 = st.columns(3)
    if c1.button("Update machine teach", key="mk_machine"):
        trace.gui_log("GUI", "menu: Update machine teach")
        st.session_state["menu"] = "marker_machine"
        # Don't reset the machine teach phase here. If a session is already
        # in progress (operator went back to the main menu mid-teach to
        # toggle drag, then came back), preserve it. The phase machine in
        # _render_machine_teach handles a stale subprocess gracefully.
        _safe_rerun()
    if c2.button("Update tool teach", key="mk_tool"):
        trace.gui_log("GUI", "menu: Update tool teach")
        st.session_state["menu"] = "marker_tool"
        _safe_rerun()
    if c3.button("Back", key="mk_back"):
        _go_to_main()
        _safe_rerun()


def _perception_stream_port() -> str:
    """Resolve the perception_streamer HTTP port for this robot.

    The streamer is launched by robot{1,2}-startup.sh and binds
    ``$PERCEPTION_STREAM_PORT`` on the pod's host network (8181 for
    robot 1, 8182 for robot 2). Since the pods use ``hostNetwork: true``,
    that port is reachable directly from the operator's browser as
    ``http://<NUC-IP>:<port>/stream``.
    """
    port = os.environ.get("PERCEPTION_STREAM_PORT")
    if port:
        return port.strip()
    robot_id = os.environ.get("ROBOT_ID", "1").strip()
    return "8181" if robot_id == "1" else "8182"


def _render_perception_stream_embed() -> None:
    """Embed the perception_streamer MJPEG into the GUI.

    The URL must be built browser-side because Streamlit doesn't know
    which hostname the operator typed into their address bar (could be
    the NUC IP, ``localhost``, a port-forward, etc.). We use a tiny
    JavaScript snippet to read ``window.location.hostname`` and point an
    ``<img>`` at ``<proto>//<host>:<port>/stream``. The streamer
    auto-enables aruco visualization on the first client connect, so
    just rendering this image turns on the labeled overlay.
    """
    import streamlit.components.v1 as components  # local import: only used here

    port = _perception_stream_port()
    components.html(
        f"""
        <div style="font-family:sans-serif;color:#aaa;font-size:0.85em;">
          MJPEG from perception_streamer (port {port}). Marker labels
          appear over each frame.
        </div>
        <img id="oms-perception-stream" alt="connecting to camera..."
             style="width:100%;border:1px solid #444;margin-top:6px;"/>
        <div id="oms-perception-fallback"
             style="font-family:monospace;color:#aaa;margin-top:6px;
                    font-size:0.85em;"></div>
        <script>
          (function() {{
            const port = "{port}";
            const host = window.location.hostname || "localhost";
            const proto = window.location.protocol || "http:";
            const url = `${{proto}}//${{host}}:${{port}}/stream`;
            const img = document.getElementById("oms-perception-stream");
            const fallback = document.getElementById("oms-perception-fallback");
            img.src = url;
            fallback.textContent = "If the image is broken, open " + url +
              " directly in another tab.";
          }})();
        </script>
        """,
        height=520,
    )


def _render_perception_status() -> None:
    pm = _proc_mgr()
    cols = st.columns(2)
    if cols[0].button("Start camera", key="proc_camera"):
        pm.ensure_camera()
    if cols[1].button("Start perception", key="proc_perception"):
        pm.ensure_perception(visualization=True)

    status = pm.status()
    for key, info in status.items():
        if key == process_manager.RVIZ_PROCESS_KEY:
            continue
        running = info["running"]
        owned = info["started_by_us"]
        if running:
            st.success(
                f"{key}: running "
                f"({'owned' if owned else 'external'})"
            )
        else:
            st.warning(f"{key}: not running")
        with st.expander(f"{key} log tail"):
            for line in info.get("tail", []):
                st.text(line)

    st.markdown("---")
    st.subheader("Camera + ArUco markers")
    st.caption(
        "Live MJPEG stream from the perception_streamer the robot pod "
        "already runs (same source the dashboard uses). RViz is "
        "intentionally not offered: the K8s pod has no X display, so "
        "rviz2 fails with `Couldn't open X display :99` -- but you don't "
        "need it because this stream already shows labeled aruco markers "
        "overlaid on the color frame."
    )
    _render_perception_stream_embed()


def _get_tool_session():
    return st.session_state.get(_TOOL_SESSION_KEY)


def _set_tool_session(session) -> None:
    st.session_state[_TOOL_SESSION_KEY] = session


def _get_machine_session():
    return st.session_state.get(_MACHINE_SESSION_KEY)


def _set_machine_session(session) -> None:
    st.session_state[_MACHINE_SESSION_KEY] = session


def _render_tool_teach() -> None:
    st.header("Tool teach")
    _render_perception_status()

    available = marker_catalog.tool_markers()
    marker = st.selectbox("Tool marker", available, key="tool_marker_select")
    st.caption(
        "Move the robot/gripper so the tool's marker is in view, then click "
        "Train. The teach node will sample averaged transforms and update "
        "tool_offset_points.yaml in both the BARNS source share and the "
        "install share, with a backup."
    )

    cols = st.columns(3)
    if cols[0].button("Train", key="tool_train"):
        trace.gui_log("GUI", f"tool_teach: train start marker={marker}")
        session = teach_runner.run_tool_teach(marker, timeout_sec=180.0)
        _set_tool_session(session)
        st.session_state["tool_session_marker"] = marker
        _safe_rerun()
    if cols[1].button("Back", key="tool_back"):
        st.session_state["menu"] = "marker_training"
        _safe_rerun()
    if cols[2].button("Refresh", key="tool_refresh"):
        _safe_rerun()

    session = _get_tool_session()
    if session is not None:
        st.subheader("Last train output")
        for line in session.tail(80):
            st.text(line)
        if session.saved_paths:
            st.success("Saved to:")
            for p in session.saved_paths:
                st.code(p)


# ---------------------------------------------------------------------------
# Machine teach: phase-based flow.
#
#     idle -> marker_ready -> approach_pending -> mount_pending -> point_done
#                                                                  |
#                                                                  +--> marker_ready (next point)
#                                                                  |
#                                                                  +--> finalize -> idle
#
# Operator can drive sidebar drag/gripper between any two phases. The
# machine_mount_teach subprocess just sits at input() until the GUI sends
# the next response.
# ---------------------------------------------------------------------------


def _machine_session_alive(session) -> bool:
    return session is not None and session.is_running()


def _render_machine_teach_session_panel() -> None:
    session = _get_machine_session()
    if session is None:
        return
    with st.expander("Machine teach log tail", expanded=False):
        for line in session.tail(120):
            st.text(line)


def _render_machine_teach() -> None:
    st.header("Machine teach")
    _render_perception_status()

    phase = st.session_state.get("machine_phase", "idle")
    session = _get_machine_session()

    # Normalize phase if the subprocess died unexpectedly.
    if phase != "idle" and not _machine_session_alive(session):
        st.warning(
            "Machine teach subprocess is no longer running -- resetting to idle."
        )
        st.session_state["machine_phase"] = "idle"
        _set_machine_session(None)
        phase = "idle"

    # ---- Phase: idle -- pick a marker and sample it ----
    if phase == "idle":
        markers = marker_catalog.machine_markers()
        marker = st.selectbox(
            "Machine marker", markers, key="machine_marker_select"
        )
        st.caption(
            "Move the robot so the marker is comfortably in the camera view, "
            "then click `Sample marker`. The teach node will average the "
            "marker pose for ~10 s before you can start adding points."
        )
        cols = st.columns(2)
        if cols[0].button("Sample marker", key="machine_sample"):
            trace.gui_log("GUI", f"machine_teach: sampling marker={marker}")
            session = teach_runner.run_machine_marker_capture(marker)
            if not _machine_session_alive(session):
                st.error(
                    "machine_mount_teach subprocess exited during averaging; "
                    "see log tail."
                )
                _set_machine_session(session)
                _render_machine_teach_session_panel()
                return
            _set_machine_session(session)
            st.session_state["machine_marker"] = marker
            st.session_state["machine_phase"] = "marker_ready"
            st.session_state["machine_captured_points"] = []
            _safe_rerun()
        if cols[1].button("Back", key="machine_back_idle"):
            st.session_state["menu"] = "marker_training"
            _safe_rerun()
        return

    marker = st.session_state.get("machine_marker") or "(unknown)"
    st.success(f"Marker `{marker}` averaged. Phase: **{phase}**")
    captured = st.session_state.get("machine_captured_points", [])
    if captured:
        st.info("Already captured this session: " + ", ".join(captured))

    # ---- Phase: marker_ready -- pick a point name and start it ----
    if phase == "marker_ready":
        candidates = marker_catalog.machine_points(marker)
        if candidates:
            point_choice = st.selectbox(
                "Point name (catalogued)",
                candidates,
                key="machine_point_select",
            )
            point_text = st.text_input(
                "Or enter a custom point name (overrides selection)",
                value="",
                key="machine_point_text",
            )
            point_name = point_text.strip() or point_choice
        else:
            point_name = st.text_input(
                f"Point name (no catalog entries for {marker!r})",
                value="",
                key="machine_point_text_only",
            ).strip()

        st.caption(
            "Click `Start point` to commit this point name. The teach node "
            "will then wait for the approach pose."
        )
        cols = st.columns(3)
        if cols[0].button("Start point", key="machine_start_point"):
            if not point_name:
                st.error("Point name cannot be empty.")
                return
            ok = teach_runner.send_machine_point_name(session, point_name)
            if not ok:
                st.error("Failed to send point name to teach subprocess.")
                return
            st.session_state["machine_current_point"] = point_name
            st.session_state["machine_phase"] = "approach_pending"
            trace.gui_log(
                "GUI", f"machine_teach: started point name={point_name}"
            )
            _safe_rerun()
        if cols[1].button("Finalize and save YAML", key="machine_finalize_a"):
            ok = teach_runner.finalize_machine_teach(session, timeout_sec=120.0)
            if ok and session.saved_paths:
                st.success("Saved to:")
                for p in session.saved_paths:
                    st.code(p)
                trace.gui_log(
                    "GUI",
                    "machine_teach: finalize ok paths="
                    + ", ".join(session.saved_paths),
                )
                st.session_state["machine_phase"] = "idle"
                _set_machine_session(None)
            else:
                st.error("Finalize did not produce a saved-file line; see log tail.")
        if cols[2].button("Discard session", key="machine_discard"):
            teach_runner.end_machine_session(session)
            _set_machine_session(None)
            st.session_state["machine_phase"] = "idle"
            st.warning("Session ended without saving.")
            trace.gui_log("GUI", "machine_teach: session discarded")
            _safe_rerun()
        _render_machine_teach_session_panel()
        return

    # ---- Phase: approach_pending -- operator moves robot, then captures ----
    if phase == "approach_pending":
        point_name = st.session_state.get("machine_current_point") or ""
        st.markdown(
            f"**Point**: `{point_name}` -- now move the robot to the "
            "**approach** pose using the sidebar drag/gripper, then click "
            "`Capture approach` below."
        )
        approach_tcp = st.text_input(
            "Approach TCP frame",
            value=st.session_state.get("machine_approach_tcp", "Link6"),
            key="machine_approach_tcp_input",
        )
        st.session_state["machine_approach_tcp"] = approach_tcp
        cols = st.columns(3)
        if cols[0].button("Capture approach", key="machine_capture_approach"):
            ok = teach_runner.capture_machine_approach(
                session, point_name, approach_tcp, timeout_sec=60.0
            )
            if ok:
                st.success(f"Captured approach pose for {point_name}.")
                trace.gui_log(
                    "GUI",
                    f"machine_teach: approach captured point={point_name} "
                    f"tcp={approach_tcp}",
                )
                st.session_state["machine_phase"] = "mount_pending"
                _safe_rerun()
            else:
                st.error(
                    "Approach capture failed -- check log tail. The subprocess "
                    "is still alive; you can move the robot and try again."
                )
        if cols[1].button("Restart this point", key="machine_restart_point_a"):
            # Send a blank to terminate this point's prompt sequence in the
            # teach node? The teach node's prompt_and_sample loops on
            # invalid frames forever. Best we can do is discard the session
            # entirely. Safer: just go back to marker_ready and let the
            # operator pick a new point name; the teach node is still at
            # the approach prompt, which the next point name path will
            # not resolve cleanly. Keep it simple: discard.
            teach_runner.end_machine_session(session)
            _set_machine_session(None)
            st.session_state["machine_phase"] = "idle"
            st.warning(
                "Restart requires a fresh subprocess; session ended. "
                "Sample marker again and start the point fresh."
            )
            _safe_rerun()
        if cols[2].button("Discard session", key="machine_discard_a"):
            teach_runner.end_machine_session(session)
            _set_machine_session(None)
            st.session_state["machine_phase"] = "idle"
            _safe_rerun()
        _render_machine_teach_session_panel()
        return

    # ---- Phase: mount_pending -- operator moves robot, then captures ----
    if phase == "mount_pending":
        point_name = st.session_state.get("machine_current_point") or ""
        st.markdown(
            f"**Point**: `{point_name}` -- approach captured. "
            "Now move the robot to the **mount** (grab) pose using the "
            "sidebar drag/gripper, then click `Capture mount` below."
        )
        mount_tcp = st.text_input(
            "Mount TCP frame",
            value=st.session_state.get("machine_mount_tcp", "Link6"),
            key="machine_mount_tcp_input",
        )
        st.session_state["machine_mount_tcp"] = mount_tcp
        cols = st.columns(2)
        if cols[0].button("Capture mount", key="machine_capture_mount"):
            ok = teach_runner.capture_machine_mount(
                session, point_name, mount_tcp, timeout_sec=60.0
            )
            if ok:
                captured = list(st.session_state.get("machine_captured_points", []))
                if point_name not in captured:
                    captured.append(point_name)
                st.session_state["machine_captured_points"] = captured
                st.success(f"Captured mount pose for {point_name}.")
                trace.gui_log(
                    "GUI",
                    f"machine_teach: mount captured point={point_name} "
                    f"tcp={mount_tcp}",
                )
                st.session_state["machine_phase"] = "point_done"
                _safe_rerun()
            else:
                st.error(
                    "Mount capture failed -- check log tail. The subprocess "
                    "is still alive; you can move the robot and try again."
                )
        if cols[1].button("Discard session", key="machine_discard_m"):
            teach_runner.end_machine_session(session)
            _set_machine_session(None)
            st.session_state["machine_phase"] = "idle"
            _safe_rerun()
        _render_machine_teach_session_panel()
        return

    # ---- Phase: point_done -- next point or finalize ----
    if phase == "point_done":
        point_name = st.session_state.get("machine_current_point") or ""
        st.success(f"Point `{point_name}` complete. What next?")
        cols = st.columns(3)
        if cols[0].button("Train another point", key="machine_train_more"):
            st.session_state["machine_phase"] = "marker_ready"
            st.session_state["machine_current_point"] = None
            _safe_rerun()
        if cols[1].button("Finalize and save YAML", key="machine_finalize_b"):
            ok = teach_runner.finalize_machine_teach(session, timeout_sec=120.0)
            if ok and session.saved_paths:
                st.success("Saved to:")
                for p in session.saved_paths:
                    st.code(p)
                trace.gui_log(
                    "GUI",
                    "machine_teach: finalize ok paths="
                    + ", ".join(session.saved_paths),
                )
                st.session_state["machine_phase"] = "idle"
                _set_machine_session(None)
            else:
                st.error(
                    "Finalize did not produce a saved-file line; see log tail."
                )
        if cols[2].button("Discard session", key="machine_discard_done"):
            teach_runner.end_machine_session(session)
            _set_machine_session(None)
            st.session_state["machine_phase"] = "idle"
            _safe_rerun()
        _render_machine_teach_session_panel()
        return

    st.error(f"Unknown machine_phase: {phase!r}; resetting.")
    st.session_state["machine_phase"] = "idle"


# ---------------------------------------------------------------------------
# trace panel
# ---------------------------------------------------------------------------


def _render_trace() -> None:
    st.markdown("---")
    st.subheader("Trace")
    items = trace.snapshot(limit=200)
    rendered = trace.render_lines(items)
    if not rendered:
        st.caption("(no trace lines yet)")
        return
    txt = "\n".join(rendered[-200:])
    st.text_area("recent log", value=txt, height=240, disabled=True, key="trace_text")
    cols = st.columns(2)
    if cols[0].button("Clear trace", key="trace_clear"):
        trace.clear()
    if cols[1].button("Refresh trace", key="trace_refresh"):
        _safe_rerun()


# ---------------------------------------------------------------------------
# router
# ---------------------------------------------------------------------------


def _route() -> None:
    menu = st.session_state.get("menu", "main")
    if menu == "main":
        _render_main_menu()
    elif menu == "update_points":
        _render_update_points_index()
    elif menu == "update_points_variant":
        _render_update_points_variant()
    elif menu == "update_points_edit":
        _render_update_points_edit()
    elif menu == "update_points_capture":
        _render_update_points_capture()
    elif menu == "update_points_test_play":
        _render_update_points_test_play()
    elif menu == "marker_training":
        _render_marker_menu()
    elif menu == "marker_tool":
        _render_tool_teach()
    elif menu == "marker_machine":
        _render_machine_teach()
    else:
        st.error(f"Unknown menu state: {menu}")
        if st.button("Reset", key="bad_reset"):
            _go_to_main()
            _safe_rerun()


def main() -> None:
    st.set_page_config(page_title=PAGE_TITLE, layout="wide")
    _ensure_state()
    st.title(PAGE_TITLE)
    _render_sidebar()
    _route()
    _render_trace()


def cli_main() -> None:
    """Console-script entrypoint -- execs ``streamlit run`` on this file.

    Allows ``oms_v1_gui`` to behave like the standard ``streamlit run`` flow
    after ``colcon build`` has installed the package.

    Also prepares a non-interactive Streamlit environment so the script
    works under ``kubectl exec`` / ``ros2 run`` (no TTY on stdin):

        - Pre-creates ``~/.streamlit/credentials.toml`` with an empty
          ``[general] email`` so Streamlit's first-run email prompt does
          not block on ``input()``.
        - Forces ``--server.headless=true`` and disables usage-stats
          telemetry by default. Operator-supplied flags still win because
          they're appended after the defaults and Streamlit's argparse
          uses last-wins semantics.
    """
    import os
    import sys

    cred_dir = Path(os.path.expanduser("~/.streamlit"))
    cred_file = cred_dir / "credentials.toml"
    if not cred_file.exists():
        try:
            cred_dir.mkdir(parents=True, exist_ok=True)
            cred_file.write_text('[general]\nemail = ""\n')
        except OSError:
            pass

    from streamlit.web import cli as stcli

    target = str(Path(__file__).resolve())
    default_flags = [
        "--server.headless=true",
        "--browser.gatherUsageStats=false",
    ]
    sys.argv = ["streamlit", "run", target, *default_flags, *sys.argv[1:]]
    sys.exit(stcli.main())


if __name__ == "__main__":
    main()
