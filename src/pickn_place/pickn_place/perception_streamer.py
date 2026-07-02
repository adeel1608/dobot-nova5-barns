#!/usr/bin/env python3
"""
Per-robot ROS-to-MJPEG bridge for the ArUco perception visualization.

Why this lives inside the robot pod
-----------------------------------
Robot 1 and Robot 2 run in their own ROS_DOMAIN_IDs (0 and 1) on different
hostNetwork pods. A single ROS subscriber outside the pod cannot see both
graphs without painful multi-domain plumbing. By running this small bridge
inside each robot pod, every node here only ever talks to its own local
DDS, and the cluster-side video-stream service just reverse-proxies HTTP.

Control plane / data plane split
--------------------------------
- Control plane: std_srvs/srv/SetBool calls to the aruco perception node's
  ~/set_visualization service. The aruco node only renders + JPEG-encodes
  while the flag is True, so disabling it returns the robot CPU to baseline.

- Data plane: this node subscribes to the aruco node's
  ~/visualization/compressed CompressedImage topic, caches the latest JPEG
  in memory, and serves an MJPEG (multipart/x-mixed-replace) HTTP stream
  on a configurable port. The stream is served with Python stdlib only
  (http.server.ThreadingHTTPServer) so we add zero pip dependencies.

HTTP endpoints
--------------
- GET  /stream            multipart MJPEG. Auto-enables visualization on the
                          first concurrent client and counts down on
                          disconnect; if no clients remain for
                          idle_grace_seconds the visualization is disabled.
- POST /control/enable    explicit SetBool(true) call. Useful for warmup.
- POST /control/disable   explicit SetBool(false) call.
- GET  /status            JSON: {robot_id, enabled, clients,
                          last_frame_age_ms, ...}

The endpoint shapes are intentionally tiny so the cluster-side video-stream
service can wrap them with the dashboard's existing UX (start/stop/status).
"""
import json
import os
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import SetBool


_MJPEG_BOUNDARY = "frame"


class FrameCache:
    """Thread-safe holder for the most recent JPEG payload."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._jpeg: Optional[bytes] = None
        self._ts: float = 0.0
        self._cv = threading.Condition(self._lock)

    def set(self, jpeg: bytes) -> None:
        with self._cv:
            self._jpeg = jpeg
            self._ts = time.time()
            self._cv.notify_all()

    def get(self) -> tuple[Optional[bytes], float]:
        with self._lock:
            return self._jpeg, self._ts

    def wait_for_new(self, last_seen_ts: float, timeout: float) -> tuple[Optional[bytes], float]:
        """Block until a fresher frame than last_seen_ts arrives or timeout."""
        deadline = time.time() + timeout
        with self._cv:
            while self._ts <= last_seen_ts:
                remaining = deadline - time.time()
                if remaining <= 0:
                    return self._jpeg, self._ts
                self._cv.wait(timeout=remaining)
            return self._jpeg, self._ts


class ClientCounter:
    """Tracks active MJPEG clients and the last time the count was non-zero."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._count = 0
        self._last_nonzero_ts = 0.0

    def inc(self) -> int:
        with self._lock:
            self._count += 1
            self._last_nonzero_ts = time.time()
            return self._count

    def dec(self) -> int:
        with self._lock:
            if self._count > 0:
                self._count -= 1
            self._last_nonzero_ts = time.time()
            return self._count

    @property
    def count(self) -> int:
        with self._lock:
            return self._count

    @property
    def idle_for(self) -> float:
        with self._lock:
            if self._count > 0:
                return 0.0
            return time.time() - self._last_nonzero_ts


class PerceptionStreamerNode(Node):
    def __init__(self) -> None:
        super().__init__('perception_streamer_node')

        # robot_id uses dynamic_typing so callers can pass it as either an
        # integer (`-p robot_id:=1`) or a string (`-p robot_id:='1'`). Older
        # versions of these launch lines did the former, and rclpy will not
        # silently coerce INT into a STRING-typed parameter.
        robot_id_descriptor = ParameterDescriptor(dynamic_typing=True)

        self.declare_parameter('aruco_node_name',     'aruco_perception_node')
        self.declare_parameter('port',                int(os.environ.get('PERCEPTION_STREAM_PORT', '8181')))
        self.declare_parameter('bind_host',           '0.0.0.0')
        self.declare_parameter('idle_grace_seconds',  3.0)
        self.declare_parameter(
            'robot_id', os.environ.get('ROBOT_ID', '0'), robot_id_descriptor
        )
        self.declare_parameter('client_read_timeout', 5.0)

        self.aruco_node_name   = str(self.get_parameter('aruco_node_name').value)
        self.port              = int(self.get_parameter('port').value)
        self.bind_host         = str(self.get_parameter('bind_host').value)
        self.idle_grace        = float(self.get_parameter('idle_grace_seconds').value)
        self.robot_id          = str(self.get_parameter('robot_id').value)
        self.client_read_timeout = float(self.get_parameter('client_read_timeout').value)

        self.get_logger().info(
            f"perception_streamer for robot_id={self.robot_id} -> "
            f"aruco={self.aruco_node_name}, http={self.bind_host}:{self.port}, "
            f"idle_grace={self.idle_grace}s"
        )

        # Subscribe to the aruco node's visualization topic. The aruco node
        # only publishes while its visualization flag is True, so when nothing
        # is listening we incur zero subscriber cost beyond the empty queue.
        viz_topic = f'/{self.aruco_node_name}/visualization/compressed'
        viz_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._frame_cache = FrameCache()
        self.create_subscription(CompressedImage, viz_topic, self._on_frame, viz_qos)

        # Service client used to drive the aruco node visualization on/off.
        # Reentrant group is required because we issue calls from the HTTP
        # server thread (and the watchdog thread) while the executor is also
        # processing the response: the default mutually-exclusive group would
        # deadlock the future and the dashboard would never see frames.
        self._service_cb_group = ReentrantCallbackGroup()
        self._set_visualization_cli = self.create_client(
            SetBool,
            f'/{self.aruco_node_name}/set_visualization',
            callback_group=self._service_cb_group,
        )

        self._enabled_state = False
        self._enable_lock = threading.Lock()
        # Note: the attribute name is intentionally NOT `self._clients`. That
        # name is used by rclpy.Node for the list of service clients; shadowing
        # it crashes the executor with `TypeError: not iterable` because
        # `Node.clients` property iterates `self._clients`.
        self._client_counter = ClientCounter()

        # Background watchdog: if no clients for idle_grace seconds while the
        # aruco node still has visualization enabled, ask it to disable.
        self._stop_event = threading.Event()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, name='perception-watchdog', daemon=True
        )
        self._watchdog_thread.start()

        # HTTP server thread.
        self._http_server = self._build_http_server()
        self._http_thread = threading.Thread(
            target=self._http_server.serve_forever,
            name=f'perception-http-{self.port}',
            daemon=True,
        )
        self._http_thread.start()
        self.get_logger().info(
            f"MJPEG endpoint live at http://{self.bind_host}:{self.port}/stream"
        )

    # ---- ROS callbacks ------------------------------------------------------
    def _on_frame(self, msg: CompressedImage) -> None:
        if not msg.data:
            return
        self._frame_cache.set(bytes(msg.data))

    # ---- Visualization control --------------------------------------------
    def _set_visualization(self, enabled: bool) -> bool:
        """Send a SetBool request to the aruco node without blocking.

        We deliberately do NOT busy-wait on the response. The HTTP handler
        thread that drives this call must return quickly so the MJPEG body
        starts streaming; the future is resolved by the executor and a
        done_callback updates state / logs the outcome. Optimistically
        applying `_enabled_state` here keeps the watchdog from immediately
        toggling visualization off while a request is in flight.
        """
        with self._enable_lock:
            if not self._set_visualization_cli.service_is_ready():
                # Best-effort discovery probe so a freshly started aruco node
                # gets a chance to register before we give up.
                if not self._set_visualization_cli.wait_for_service(timeout_sec=0.5):
                    self.get_logger().warn(
                        f"set_visualization service not yet available "
                        f"({self._set_visualization_cli.srv_name})"
                    )
                    return False

            req = SetBool.Request()
            req.data = bool(enabled)
            self._enabled_state = bool(enabled)
            future = self._set_visualization_cli.call_async(req)
            future.add_done_callback(
                lambda fut, want=enabled: self._on_set_viz_response(fut, want)
            )
            self.get_logger().info(
                f"set_visualization({enabled}) request dispatched"
            )
            return True

    def _on_set_viz_response(self, future, requested_state: bool) -> None:
        """Reconcile the optimistic state once the aruco node replies."""
        try:
            resp = future.result()
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().warn(
                f"set_visualization({requested_state}) failed: {exc}"
            )
            return
        if resp is None:
            self.get_logger().warn(
                f"set_visualization({requested_state}) returned no response"
            )
            return
        if not resp.success:
            self.get_logger().warn(
                f"set_visualization({requested_state}) refused: {resp.message}"
            )
            return
        self.get_logger().info(
            f"aruco visualization confirmed -> {requested_state} ({resp.message})"
        )

    def _watchdog_loop(self) -> None:
        while not self._stop_event.is_set():
            time.sleep(1.0)
            try:
                if (
                    self._enabled_state
                    and self._client_counter.count == 0
                    and self._client_counter.idle_for >= self.idle_grace
                ):
                    self.get_logger().info(
                        f"No clients for {self.idle_grace}s; disabling visualization"
                    )
                    self._set_visualization(False)
            except Exception as exc:  # pragma: no cover - defensive
                self.get_logger().error(f"watchdog error: {exc}")

    # ---- HTTP server -------------------------------------------------------
    def _build_http_server(self) -> ThreadingHTTPServer:
        node = self

        class Handler(BaseHTTPRequestHandler):
            # Silence the default access log; we route through ROS logging.
            def log_message(self, format: str, *args) -> None:  # noqa: A003
                node.get_logger().debug("HTTP " + (format % args))

            def do_GET(self) -> None:  # noqa: N802
                if self.path.startswith('/stream'):
                    return self._serve_stream()
                if self.path.startswith('/status'):
                    return self._serve_status()
                self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

            def do_POST(self) -> None:  # noqa: N802
                if self.path.startswith('/control/enable'):
                    return self._serve_control(True)
                if self.path.startswith('/control/disable'):
                    return self._serve_control(False)
                self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

            # ---- helpers -----------------------------------------------------
            def _send_json(self, status: HTTPStatus, body: dict) -> None:
                payload = json.dumps(body).encode('utf-8')
                self.send_response(status)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(payload)))
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(payload)

            def _serve_status(self) -> None:
                jpeg, ts = node._frame_cache.get()
                last_frame_age_ms = (
                    int((time.time() - ts) * 1000) if ts else None
                )
                self._send_json(HTTPStatus.OK, {
                    "robot_id": node.robot_id,
                    "enabled": node._enabled_state,
                    "clients": node._client_counter.count,
                    "has_frame": jpeg is not None,
                    "last_frame_age_ms": last_frame_age_ms,
                    "aruco_node": node.aruco_node_name,
                })

            def _serve_control(self, want_enabled: bool) -> None:
                ok = node._set_visualization(want_enabled)
                self._send_json(
                    HTTPStatus.OK if ok else HTTPStatus.SERVICE_UNAVAILABLE,
                    {
                        "robot_id": node.robot_id,
                        "requested": want_enabled,
                        "enabled": node._enabled_state,
                        "ok": ok,
                    },
                )

            def _serve_stream(self) -> None:
                # Auto-enable visualization on first client connect.
                node._client_counter.inc()
                try:
                    if not node._enabled_state:
                        node._set_visualization(True)

                    self.send_response(HTTPStatus.OK)
                    self.send_header('Content-Type',
                                     f'multipart/x-mixed-replace; boundary={_MJPEG_BOUNDARY}')
                    self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
                    self.send_header('Pragma', 'no-cache')
                    self.send_header('Connection', 'close')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()

                    last_ts = 0.0
                    while not node._stop_event.is_set():
                        jpeg, ts = node._frame_cache.wait_for_new(
                            last_ts, timeout=node.client_read_timeout
                        )
                        if jpeg is None or ts <= last_ts:
                            # Fresh frame did not arrive in time. Send a 1-byte
                            # comment chunk to keep the connection warm so
                            # browsers do not close the <img> tag.
                            try:
                                self.wfile.write(b'--' + _MJPEG_BOUNDARY.encode() + b'\r\n')
                                self.wfile.write(b'Content-Type: text/plain\r\n')
                                self.wfile.write(b'Content-Length: 0\r\n\r\n\r\n')
                                self.wfile.flush()
                            except (BrokenPipeError, ConnectionResetError):
                                break
                            continue

                        last_ts = ts
                        try:
                            self.wfile.write(b'--' + _MJPEG_BOUNDARY.encode() + b'\r\n')
                            self.wfile.write(b'Content-Type: image/jpeg\r\n')
                            self.wfile.write(
                                f'Content-Length: {len(jpeg)}\r\n\r\n'.encode()
                            )
                            self.wfile.write(jpeg)
                            self.wfile.write(b'\r\n')
                            self.wfile.flush()
                        except (BrokenPipeError, ConnectionResetError):
                            break
                finally:
                    node._client_counter.dec()

        server = ThreadingHTTPServer((self.bind_host, self.port), Handler)
        server.daemon_threads = True
        return server

    # ---- Lifecycle ---------------------------------------------------------
    def destroy_node(self) -> None:
        self.get_logger().info("Shutting down perception_streamer_node...")
        self._stop_event.set()
        try:
            self._set_visualization(False)
        except Exception:
            pass
        try:
            self._http_server.shutdown()
            self._http_server.server_close()
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().warn(f"HTTP server shutdown error: {exc}")
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionStreamerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
