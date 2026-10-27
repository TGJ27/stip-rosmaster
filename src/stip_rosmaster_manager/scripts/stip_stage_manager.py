#!/usr/bin/env python3
"""STIP ROSMASTER staged startup, managed by stip_rosmaster_manager.

Launches are separate OS process groups. Bringup never auto-restarts.
Non-bringup modules are retried independently; an error does not kill others.
The manager's own availability, ROS master, and shared hardware remain
common dependencies and must be supervised at the deployment level.
"""
import json
import os
import re
import signal
import subprocess
import threading
import time
from pathlib import Path

import rospy
import rospkg
import rosnode
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, TriggerResponse

# Optional device stages never gate robot readiness / purple LED.
CORE_STAGES = ("ui", "bringup", "camera", "qr", "fire")
OPTIONAL_STAGES = ("gas", "thermal")
STAGES = CORE_STAGES + OPTIONAL_STAGES
# At most THREE unsuccessful start attempts per recovery cycle for these stages.
LIMITED_STAGES = ("qr", "fire", "gas", "thermal")
NODES = {"ui": "/stip_inspection_ui",
         "bringup": "/driver_node",
         "camera": "/stip_inspection_camera",
         "qr": "/stip_qr_inspection",
         "fire": "/stip_fire_detector_cpp",
         "gas": "/stip_gas_sensor",
         "thermal": "/stip_thermal_camera"}

class Manager:
    def __init__(self):
        self.lock = threading.RLock()
        self.processes = {}
        self.states = {m: "WAITING" for m in STAGES}
        self.details = {m: "" for m in STAGES}
        self.retry_requested = threading.Event()
        self.stopped = threading.Event()
        # An intentional UI exit is distinct from an unexpected UI crash.
        # The event is set BEFORE the shutdown RPC is acknowledged.
        self._shutdown_lock = threading.Lock()
        self._shutdown_completed = False
        self.restart_requests = set()
        self.bringup_status = ""
        self.bringup_msg_time = 0.0
        self.bringup_ready = False
        self.ui_ready = False
        self.ui_frame_ready = False
        self.camera_valid = False
        self.camera_seen_time = 0.0
        self.retry_delay = max(1.0, float(rospy.get_param("~retry_delay", 5.0)))
        self.warmup_timeout = float(rospy.get_param("~warmup_timeout", 12.0))
        self.bringup_timeout = float(rospy.get_param("~bringup_timeout", 25.0))
        self.max_start_attempts = max(1, int(rospy.get_param("~max_start_attempts", 3)))
        self.optional_health = {m: (False, 0.0) for m in OPTIONAL_STAGES}
        self.optional_max_age = {"gas": 5.0, "thermal": 8.0}
        # Two separate workers: unplugged gas never delays thermal, or vice versa.
        self.optional_threads = {
            stage: threading.Thread(target=self.optional_supervise, args=(stage,),
                                    name="stip_" + stage + "_supervisor", daemon=True)
            for stage in OPTIONAL_STAGES
        }
        self.qr_launch_dir = Path(rospkg.RosPack().get_path("stip_qr_inspection")) / "launch"
        self.fire_launch_dir = Path(rospkg.RosPack().get_path("stip_fire_detection")) / "launch"
        self.boot_dir = Path(rospkg.RosPack().get_path("stip_rosmaster_bringup")) / "launch"
        self.cam_dir = Path(rospkg.RosPack().get_path("stip_inspection_camera")) / "launch"
        self.gas_dir = Path(rospkg.RosPack().get_path("stip_gas_sensor")) / "launch"
        self.thermal_dir = Path(rospkg.RosPack().get_path("stip_thermal_camera")) / "launch"
        self.initial_fire = bool(rospy.get_param("~fire_initially_enabled", False))
        # Warmup always happens while fire is OFF. User can activate afterward.
        if self.initial_fire:
            rospy.logwarn("v11 startup overrides fire_initially_enabled=true to OFF for safe one-frame warmup")
        self.launch_specs = {
            "ui": (self.qr_launch_dir / "stage_ui.launch", ["ui_decoder_backend:=" +
                   rospy.get_param("~ui_decoder_backend", "nvidia")]),
            "bringup": (self.boot_dir / "bringup.launch", [
                "driver_required:=false", "driver_respawn:=false", "aux_respawn:=false",
                "status_led_require_all_modules_ready:=true"]),
            "camera": (self.cam_dir / "camera.launch", ["camera_respawn:=false"]),
            "qr": (self.qr_launch_dir / "stage_qr.launch", ["qr_decoder_backend:=" +
                   rospy.get_param("~qr_decoder_backend", "nvidia")]),
            "fire": (self.fire_launch_dir / "stage_fire.launch", [
                "fire_decoder_backend:=" + rospy.get_param("~fire_decoder_backend", "nvidia"),
                "fire_initially_enabled:=false", "fire_respawn:=false"]),
            "gas": (self.gas_dir / "gas_sensor.launch", [
                "port:=" + str(rospy.get_param("~gas_serial_port", "/dev/ttyACM0"))]),
            "thermal": (self.thermal_dir / "thermal_camera.launch", [
                "i2c_bus:=" + str(rospy.get_param("~thermal_i2c_bus", 0)),
                "viewer:=false"]),
        }
        self.status_pub = rospy.Publisher("/stip/startup/status", String, latch=True, queue_size=1)
        self.module_pub = rospy.Publisher("/stip/startup/modules", String, latch=True, queue_size=1)
        self.ready_pub = rospy.Publisher("/stip/startup/bringup_ready", Bool, latch=True, queue_size=1)
        # Best-effort stop command; the driver also commands zero velocity in
        # its own on_shutdown handler before its serial connection closes.
        self.stop_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.all_ready_pub = rospy.Publisher("/stip/startup/all_ready", Bool,
                                             latch=True, queue_size=1)
        self.heartbeat_pub = rospy.Publisher("/stip/startup/heartbeat", Bool, queue_size=1)
        # The 1 Hz publisher refreshes the ALL_READY signal too; if the stage
        # manager stops, the robot LED watchdog withdraws PURPLE within 3 s.
        self.heartbeat_timer = rospy.Timer(rospy.Duration(1.0),
                                           self.heartbeat_tick)
        rospy.Subscriber("/bringup_status", String, self.bringup_callback, queue_size=10)
        rospy.Subscriber("/stip/inspection_ui/ready", Bool,
                         lambda msg: setattr(self, "ui_ready", bool(msg.data)), queue_size=1)
        rospy.Subscriber("/stip/inspection_ui/frame_ready", Bool,
                         lambda msg: setattr(self, "ui_frame_ready", bool(msg.data)), queue_size=1)
        rospy.Subscriber("/inspection_camera/image_raw/compressed", CompressedImage,
                         self.camera_callback, queue_size=1, buff_size=4 * 1024 * 1024)
        for module, topic in (("gas", "/gas_sensor/healthy"),
                              ("thermal", "/thermal/healthy")):
            rospy.Subscriber(topic, Bool, self.make_health_callback(module), queue_size=1)
        rospy.Service("/stip/startup/retry_bringup", Trigger, self.manual_bringup)
        rospy.Service("/stip/startup/shutdown", Trigger, self.request_shutdown)
        for stage in ("ui", "camera", "qr", "fire", "gas", "thermal"):
            rospy.Service("/stip/modules/restart_" + stage, Trigger,
                          self.make_restart_callback(stage))
        self.ready_pub.publish(Bool(data=False))
        self.all_ready_pub.publish(Bool(data=False))
        self.set_state("ui", "WAITING", "Startup beginning")
        rospy.on_shutdown(self.shutdown)

    def make_health_callback(self, stage):
        def cb(msg):
            with self.lock:
                self.optional_health[stage] = (bool(msg.data), time.monotonic())
        return cb

    def optional_is_healthy(self, stage):
        with self.lock:
            ok, stamp = self.optional_health[stage]
        return ok and (time.monotonic() - stamp < self.optional_max_age[stage])

    def heartbeat_tick(self, _event):
        if self.stopped.is_set():
            return
        self.heartbeat_pub.publish(Bool(data=True))
        self.publish_all_ready()

    def publish_all_ready(self):
        # A stage is READY only after its one-frame warmup passed. Bringup must
        # also be healthy, and a stale/error status must never turn on PURPLE.
        with self.lock:
            healthy = (self.bringup_ready and
                       all(self.states[m] == "READY" for m in CORE_STAGES) and
                       not self.stopped.is_set() and
                       not self.bringup_status.startswith("ERROR:"))
        self.all_ready_pub.publish(Bool(data=bool(healthy)))

    def set_state(self, stage, state, detail=""):
        with self.lock:
            self.states[stage], self.details[stage] = state, detail
            self.module_pub.publish(String(data=json.dumps({
                m: {"state": self.states[m], "detail": self.details[m]} for m in STAGES
            }, sort_keys=True)))
            summary = "{} {}{}".format(stage.upper(), state,
                                      (" - " + detail) if detail else "")
            self.status_pub.publish(String(data=summary))
        self.publish_all_ready()
        rospy.loginfo("STIP startup: %s", summary)

    def bringup_callback(self, msg):
        with self.lock:
            self.bringup_status = msg.data
            self.bringup_msg_time = time.monotonic()
        if msg.data.startswith("ERROR:"):
            self.all_ready_pub.publish(Bool(data=False))

    def camera_callback(self, msg):
        # Minimal health-only subscriber. Never keep/copy/decode a full JPEG
        # in the supervisor. The UI's one-frame readiness check proves that
        # NVIDIA/OpenCV can decode the actual camera stream.
        data = msg.data
        valid = (len(data) >= 4 and
                 bytes(data[:2]) == b"\xff\xd8" and
                 b"\xff\xd9" in bytes(data[-128:]))
        if valid:
            with self.lock:
                self.camera_valid = True
                self.camera_seen_time = time.monotonic()

    def initiate_shutdown(self, reason="requested"):
        # Important order: inhibit recovery and withdraw readiness FIRST.
        # Do not kill the UI here: let the Trigger RPC respond first.
        with self._shutdown_lock:
            if self.stopped.is_set():
                return False
            self.stopped.set()
            self.retry_requested.set()  # wake manual-retry wait immediately
        rospy.loginfo("STIP: graceful application shutdown (%s)", reason)
        try:
            self.status_pub.publish(String(data="SYSTEM SHUTTING_DOWN - " + reason))
            self.ready_pub.publish(Bool(data=False))
            self.all_ready_pub.publish(Bool(data=False))
        except Exception as exc:
            rospy.logwarn("Could not announce shutdown: %s", exc)
        # Zero command is best-effort over ROS; bringup driver's cancel()
        # performs the final stop directly through its own serial handle.
        try:
            self.stop_vel_pub.publish(Twist())
        except Exception as exc:
            rospy.logwarn("Could not issue zero /cmd_vel: %s", exc)
        return True

    def request_shutdown(self, _req):
        if self.initiate_shutdown("Q pressed in inspection UI"):
            return TriggerResponse(True, "Graceful shutdown accepted; recovery disabled")
        return TriggerResponse(True, "Shutdown already in progress")

    def manual_bringup(self, _req):
        if self.stopped.is_set():
            return TriggerResponse(False, "Application is shutting down")
        if self.bringup_ready:
            return TriggerResponse(False, "Bringup already READY; no restart needed")
        with self.lock:
            status = self.states["bringup"]
        if status not in ("BLOCKED", "ERROR", "WAITING"):
            return TriggerResponse(False, "Bringup is still starting; retry not allowed")
        self.retry_requested.set()
        return TriggerResponse(True, "Manual bringup retry queued. Keep robot stationary.")

    def make_restart_callback(self, stage):
        # Restart requests are serialized through a single manager thread.
        def cb(_req):
            if self.stopped.is_set():
                return TriggerResponse(False, "Application is shutting down")
            if not self.bringup_ready and stage != "ui":
                return TriggerResponse(False, "Bringup gate closed")
            with self.lock:
                self.restart_requests.add(stage)
            return TriggerResponse(True, "Restart {} queued".format(stage))
        return cb

    def alive_nodes(self):
        try:
            return set(rosnode.get_node_names())
        except Exception:
            return set()

    def start_process(self, stage):
        if self.stopped.is_set():
            raise RuntimeError("Shutdown requested; refusing to start " + stage)
        path, args = self.launch_specs[stage]
        if not path.is_file():
            raise RuntimeError("Missing launch file: " + str(path))
        env = dict(os.environ)
        env["PYTHONUNBUFFERED"] = "1"
        # Independent roslaunch process; only one stage is started at a time.
        proc = subprocess.Popen(["roslaunch", str(path)] + list(args),
                                env=env, start_new_session=True)
        with self.lock:
            self.processes[stage] = proc
        if self.stopped.is_set():
            self.stop_process(stage)
            raise RuntimeError("Shutdown interrupted launch of " + stage)
        self.set_state(stage, "STARTING", "pid={}".format(proc.pid))

    def stop_process(self, stage):
        with self.lock:
            proc = self.processes.pop(stage, None)
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=6.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=3.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        except Exception as exc:
            rospy.logwarn("Stop %s error: %s", stage, exc)

    def wait_for_node(self, stage, seconds=12):
        deadline = time.monotonic() + seconds
        while not rospy.is_shutdown() and not self.stopped.is_set() and time.monotonic() < deadline:
            proc = self.processes.get(stage)
            if proc and proc.poll() is not None:
                raise RuntimeError("{} launch exited {}".format(stage, proc.returncode))
            if NODES[stage] in self.alive_nodes():
                return
            time.sleep(0.35)
        raise RuntimeError("{} ROS node not found before timeout".format(stage))

    def camera_warmup(self):
        # This stage verifies receipt of one well-framed JPEG. The next
        # check_ui_first_frame() stage requires the UI to decode and display
        # one frame successfully, avoiding a duplicate CPU JPEG decode here.
        deadline = time.monotonic() + self.warmup_timeout
        while not rospy.is_shutdown() and not self.stopped.is_set() and time.monotonic() < deadline:
            with self.lock:
                valid = self.camera_valid
                age = time.monotonic() - self.camera_seen_time
            if valid and age < 1.0:
                return "fresh JPEG received; UI verifies decoded preview"
            self.stopped.wait(0.15)
        raise RuntimeError("No fresh JPEG received within timeout")

    def check_ui_first_frame(self):
        end = time.monotonic() + self.warmup_timeout
        while not rospy.is_shutdown() and not self.stopped.is_set() and time.monotonic() < end:
            if self.ui_frame_ready:
                return True
            time.sleep(0.15)
        return False

    def call_warmup(self, name):
        deadline = time.monotonic() + self.warmup_timeout
        while not rospy.is_shutdown() and not self.stopped.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise RuntimeError("Required warmup service unavailable: " + name)
            try:
                rospy.wait_for_service(name, timeout=min(0.5, remaining))
                break
            except rospy.ROSException:
                continue
        if self.stopped.is_set() or rospy.is_shutdown():
            raise RuntimeError("Warmup interrupted by shutdown")

        # Use an external subprocess: a hung CUDA/GStreamer Trigger handler
        # must never hang the supervisor. Poll in short intervals so Q is fast.
        proc = subprocess.Popen(["rosservice", "call", name],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                text=True, start_new_session=True)
        try:
            while not rospy.is_shutdown() and not self.stopped.is_set():
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise RuntimeError("Warmup RPC timed out: " + name)
                try:
                    out, err = proc.communicate(timeout=min(0.35, remaining))
                    output = (out + "\n" + err).strip()
                    if proc.returncode != 0 or not re.search(r"success:\s*[Tt]rue", output):
                        raise RuntimeError("Warmup RPC failed: " + output[-300:])
                    return output.replace("\n", "; ")[:160]
                except subprocess.TimeoutExpired:
                    pass
            raise RuntimeError("Warmup interrupted by shutdown")
        finally:
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    proc.communicate(timeout=0.5)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(proc.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    proc.communicate()

    def try_stage(self, stage):
        self.stop_process(stage)
        if stage in OPTIONAL_STAGES:
            with self.lock:
                self.optional_health[stage] = (False, 0.0)
        if stage == "ui":
            self.ui_ready = False
            self.ui_frame_ready = False
        if stage == "camera":
            with self.lock:
                self.camera_valid = False
                self.camera_seen_time = 0.0
        self.start_process(stage)
        self.wait_for_node(stage, self.warmup_timeout)
        if self.stopped.is_set():
            raise RuntimeError("Shutdown interrupted stage " + stage)
        if stage == "ui":
            deadline = time.monotonic() + self.warmup_timeout
            while not rospy.is_shutdown() and not self.stopped.is_set() and time.monotonic() < deadline:
                if self.ui_ready:
                    return "inspection window created (waiting for camera)"
                time.sleep(0.1)
            raise RuntimeError("UI did not confirm its window was created")
        if stage in OPTIONAL_STAGES:
            deadline = time.monotonic() + self.warmup_timeout
            while not rospy.is_shutdown() and not self.stopped.is_set() and time.monotonic() < deadline:
                if self.optional_is_healthy(stage):
                    return "first valid hardware measurement received"
                proc = self.processes.get(stage)
                if proc is not None and proc.poll() is not None:
                    raise RuntimeError(stage + " process exited before first valid measurement")
                self.stopped.wait(0.1)
            raise RuntimeError(stage + " did not report a valid measurement")
        if stage == "camera":
            return self.camera_warmup()
        if stage == "qr":
            return self.call_warmup("/inspection/warmup")
        if stage == "fire":
            return self.call_warmup("/fire_detection/warmup")
        return "node registered"

    def auto_retry_stage(self, stage):
        # A BLOCKED stage is deliberately inert until its manual restart service.
        if self.states[stage] == "BLOCKED":
            return False
        attempt = 0
        while not rospy.is_shutdown() and not self.stopped.is_set():
            if stage != "ui" and not self.bringup_ready:
                return False
            if stage not in ("ui", "bringup", "gas", "thermal"):
                with self.lock:
                    bstatus = self.bringup_status
                if bstatus.startswith("ERROR:"):
                    self.bringup_ready = False
                    self.ready_pub.publish(Bool(data=False))
                    self.set_state("bringup", "BLOCKED", "Bringup ERROR while staging; press R manually")
                    return False
            attempt += 1
            try:
                result = self.try_stage(stage)
                if self.stopped.is_set():
                    return False
                self.set_state(stage, "READY", result)
                return True
            except Exception as exc:
                if self.stopped.is_set():
                    return False
                self.set_state(stage, "ERROR", "attempt {}/{}: {}".format(
                    attempt, self.max_start_attempts if stage in LIMITED_STAGES else "unlimited", exc))
                self.stop_process(stage)
                if self.stopped.is_set():
                    return False
                if stage in LIMITED_STAGES and attempt >= self.max_start_attempts:
                    self.set_state(stage, "BLOCKED", "{} failed starts; manual restart required".format(attempt))
                    return False
                self.set_state(stage, "RETRYING", "attempt {} in {:.0f}s".format(
                    attempt + 1, self.retry_delay))
                self.stopped.wait(self.retry_delay)
        return False

    def wait_bringup(self):
        # Bringup is a single safety-critical unit: no auto-retry on ERROR.
        self.stop_process("bringup")
        with self.lock:
            self.bringup_status = ""
            self.bringup_msg_time = 0.0
        self.bringup_ready = False
        self.ready_pub.publish(Bool(data=False))
        if self.stopped.is_set():
            return False
        self.start_process("bringup")
        try:
            self.wait_for_node("bringup", self.bringup_timeout)
            deadline = time.monotonic() + self.bringup_timeout
            while not rospy.is_shutdown() and not self.stopped.is_set() and time.monotonic() < deadline:
                with self.lock:
                    status = self.bringup_status
                    age = time.monotonic() - self.bringup_msg_time
                if status.startswith("ERROR:") and age < 5:
                    raise RuntimeError(status)
                if status.startswith("READY:") and age < 5 and {
                    "/driver_node", "/odometry_publisher", "/imu_filter_madgwick",
                    "/ekf_localization", "/robot_state_publisher"
                }.issubset(self.alive_nodes()):
                    self.bringup_ready = True
                    self.ready_pub.publish(Bool(data=True))
                    self.set_state("bringup", "READY", status)
                    return True
                proc = self.processes.get("bringup")
                if proc and proc.poll() is not None:
                    raise RuntimeError("Bringup launch exited")
                time.sleep(0.25)
            raise RuntimeError("Bringup READY timeout (no safe startup)")
        except Exception as exc:
            if self.stopped.is_set():
                return False
            self.bringup_ready = False
            self.ready_pub.publish(Bool(data=False))
            self.set_state("bringup", "BLOCKED", str(exc) + "; press R manually")
            self.stop_process("bringup")
            return False

    def optional_supervise(self, stage):
        """Independent worker: sensor failure never stalls base/QR/fire or peers."""
        while not rospy.is_shutdown() and not self.stopped.is_set():
            # ROSMASTER_LAUNCHER_V1_GATED: both optional sensors wait for a
            # VERIFIED bringup READY before consuming any retry attempt.
            if not self.bringup_ready:
                if self.states[stage] in ("WAITING", "WAITING_FOR_BRINGUP"):
                    if self.states[stage] != "WAITING_FOR_BRINGUP":
                        self.set_state(stage, "WAITING_FOR_BRINGUP",
                                       "Waiting for robot bringup READY; no retries consumed")
                self.stopped.wait(0.35)
                continue
            with self.lock:
                requested = stage in self.restart_requests
                if requested:
                    self.restart_requests.remove(stage)
            if requested:
                self.set_state(stage, "RESTARTING", "Manual request (resets 3-try budget)")
                self.auto_retry_stage(stage)
            elif self.states[stage] in ("WAITING", "WAITING_FOR_BRINGUP"):
                self.auto_retry_stage(stage)
            elif self.states[stage] == "READY" and (
                NODES[stage] not in self.alive_nodes() or
                (self.processes.get(stage) and self.processes[stage].poll() is not None) or
                not self.optional_is_healthy(stage)
            ):
                self.set_state(stage, "ERROR", "missing node/process or stale measurements")
                self.auto_retry_stage(stage)
            self.stopped.wait(2.0)

    def supervise(self):
        while not rospy.is_shutdown() and not self.stopped.is_set():
            with self.lock:
                requested = self.restart_requests.intersection(CORE_STAGES)
                self.restart_requests.difference_update(requested)
            # When bringup loses health, lock motion and wait for manual retry.
            with self.lock:
                bstatus, btime = self.bringup_status, self.bringup_msg_time
            if self.bringup_ready and ((bstatus.startswith("ERROR:") and
                                       time.monotonic() - btime < 5) or
                                      not {"/driver_node", "/odometry_publisher", "/imu_filter_madgwick",
                                           "/ekf_localization", "/robot_state_publisher"}.issubset(self.alive_nodes())):
                self.bringup_ready = False
                self.ready_pub.publish(Bool(data=False))
                self.set_state("bringup", "BLOCKED", "Lost bringup health; press R manually")
                self.stop_process("bringup")
            if not self.bringup_ready:
                if self.retry_requested.wait(0.4):
                    self.retry_requested.clear()
                    if self.wait_bringup():
                        for stage in ("camera", "qr", "fire"):
                            if self.states[stage] not in ("READY", "BLOCKED"):
                                self.auto_retry_stage(stage)
                            if stage == "camera" and not self.ui_frame_ready:
                                if not self.check_ui_first_frame():
                                    self.auto_retry_stage("ui")
                continue

            for stage in CORE_STAGES:
                if stage == "bringup":
                    continue
                if stage in requested:
                    self.set_state(stage, "RESTARTING", "Manual request")
                    self.auto_retry_stage(stage)

            names = self.alive_nodes()
            for stage in ("ui", "camera", "qr", "fire"):
                if stage in requested or self.states[stage] == "BLOCKED":
                    continue
                if self.states[stage] != "READY":
                    self.auto_retry_stage(stage)
                elif (NODES[stage] not in names or
                      (self.processes.get(stage) and self.processes[stage].poll() is not None) or
                      (stage == "camera" and
                       time.monotonic() - self.camera_seen_time > 3.0)):
                    self.set_state(stage, "ERROR", "process/node unavailable")
                    self.auto_retry_stage(stage)
            self.stopped.wait(2.0)

    def run(self):
        # Existing v15 system order stays intact; independent sensors start
        # after core stages. A missing optional module never blocks READY.
        if not self.auto_retry_stage("ui"):
            return
        for thread in self.optional_threads.values():
            thread.start()
        if not self.wait_bringup():
            self.supervise()
            return
        if self.auto_retry_stage("camera"):
            while not rospy.is_shutdown() and not self.stopped.is_set():
                if self.check_ui_first_frame():
                    break
                self.set_state("ui", "ERROR", "no valid decoded preview frame")
                if not self.auto_retry_stage("ui"):
                    return
            for stage in ("qr", "fire"):
                self.auto_retry_stage(stage)  # QR failure must not skip fire
        self.supervise()

    def shutdown(self):
        # Both rospy.on_shutdown and the main-thread finally clause may call
        # this. Only one caller is allowed to tear down child process groups.
        self.initiate_shutdown("ROS shutdown / manager exiting")
        with self._shutdown_lock:
            if self._shutdown_completed:
                return
            self._shutdown_completed = True
        self.heartbeat_timer.shutdown()
        for thread in self.optional_threads.values():
            if thread.is_alive() and thread is not threading.current_thread():
                thread.join(timeout=3.0)
        try:
            self.ready_pub.publish(Bool(data=False))
            self.all_ready_pub.publish(Bool(data=False))
            self.stop_vel_pub.publish(Twist())
        except Exception:
            pass
        # Fire, QR, camera, bringup, UI. Bringup driver's cancel() stops
        # motors and turns off the LEDs. UI stays up until near the end.
        for stage in reversed(STAGES):
            self.stop_process(stage)
        rospy.loginfo("STIP: all staged processes have been asked to stop")

if __name__ == "__main__":
    rospy.init_node("stip_stage_manager", disable_signals=False)
    mgr = Manager()
    try:
        mgr.run()
    finally:
        mgr.shutdown()
