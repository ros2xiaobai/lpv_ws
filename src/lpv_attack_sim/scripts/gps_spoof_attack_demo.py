#!/usr/bin/env python3
import copy
import csv
import math
import os
import sys
import time
from xml.etree import ElementTree as ET

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, Vector3, TwistStamped
from mavros_msgs.msg import ManualControl, OverrideRCIn, ParamValue, PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandLong, ParamSet, SetMode
from tf.transformations import euler_from_quaternion, quaternion_from_euler

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from result_paths import make_run_dir


class GpsSpoofAttackDemo:
    def __init__(self):
        self.vehicle_ns = rospy.get_param("~vehicle_ns", rospy.get_param("vehicle_ns", "iris_0")).strip("/")
        self.rate_hz = float(rospy.get_param("~rate_hz", rospy.get_param("rate_hz", 30.0)))

        self.takeoff_height = float(rospy.get_param("~takeoff_height", rospy.get_param("takeoff_height", 2.0)))
        self.warmup_time = float(rospy.get_param("~warmup_time", rospy.get_param("warmup_time", 4.0)))
        self.pre_attack_time = float(rospy.get_param("~pre_attack_time", rospy.get_param("pre_attack_time", 18.0)))
        self.attack_duration = float(rospy.get_param("~attack_duration", rospy.get_param("attack_duration", 35.0)))
        self.post_attack_time = float(rospy.get_param("~post_attack_time", rospy.get_param("post_attack_time", 12.0)))

        self.spoof_offset = self._vector_param("gps_spoof_offset", {"x": 1.2, "y": 0.8, "z": 0.0})
        self.drift_rate = self._vector_param("gps_spoof_drift_rate", {"x": 0.015, "y": 0.0, "z": 0.0})

        # Smooth transition parameters for attack onset and exit.
        self.smooth_transition_duration = float(rospy.get_param(
            "~smooth_transition_duration",
            rospy.get_param("smooth_transition_duration", 3.0)
        ))
        self.smooth_exit_duration = float(rospy.get_param(
            "~smooth_exit_duration",
            rospy.get_param("smooth_exit_duration", self.smooth_transition_duration)
        ))
        self.gps_spoof_start_s = self.warmup_time + self.pre_attack_time
        self.gps_spoof_end_s = self.gps_spoof_start_s + self.attack_duration
        self.gps_spoof_takeoff_z = float(rospy.get_param(
            "~gps_spoof_takeoff_z",
            rospy.get_param("gps_spoof_takeoff_z", 1.5)
        ))
        self.gps_spoof_sdf = rospy.get_param(
            "~gps_spoof_sdf",
            rospy.get_param(
                "gps_spoof_sdf",
                "/home/lxx/PX4_Firmware/Tools/sitl_gazebo/models/gps_spoof/gps_spoof.sdf"
            )
        )
        self._load_gps_spoof_sdf_params()

        self.traj_type = rospy.get_param("~trajectory/type", rospy.get_param("trajectory/type", "circle"))
        self.center_x = float(rospy.get_param("~trajectory/center_x", rospy.get_param("trajectory/center_x", 0.0)))
        self.center_y = float(rospy.get_param("~trajectory/center_y", rospy.get_param("trajectory/center_y", 0.0)))
        self.radius = float(rospy.get_param("~trajectory/radius", rospy.get_param("trajectory/radius", 2.0)))
        self.angular_rate = float(rospy.get_param("~trajectory/angular_rate", rospy.get_param("trajectory/angular_rate", 0.12)))
        self.yaw = float(rospy.get_param("~trajectory/yaw", rospy.get_param("trajectory/yaw", 0.0)))

        self.results_dir = rospy.get_param("~results_dir", rospy.get_param("results_dir", "/tmp"))
        self.run_dir, self.run_stamp = make_run_dir(self.results_dir)
        self.csv_path = os.path.join(self.run_dir, "gps_spoof_attack_%s.csv" % self.run_stamp)
        rospy.loginfo("GPS spoof results directory: %s", self.run_dir)

        topic_root = "/" + self.vehicle_ns
        self.state_sub = rospy.Subscriber(topic_root + "/mavros/state", State, self._state_cb, queue_size=1)
        self.pose_sub = rospy.Subscriber(topic_root + "/mavros/local_position/pose", PoseStamped, self._pose_cb, queue_size=1)
        self.vel_sub = rospy.Subscriber(topic_root + "/mavros/local_position/velocity_local", TwistStamped, self._vel_cb, queue_size=1)
        self.model_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states_cb, queue_size=1)

        self.setpoint_pub = rospy.Publisher(topic_root + "/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.raw_setpoint_pub = rospy.Publisher(topic_root + "/mavros/setpoint_raw/local", PositionTarget, queue_size=1)
        self.manual_pub = rospy.Publisher(topic_root + "/mavros/manual_control/send", ManualControl, queue_size=1)
        self.rc_override_pub = rospy.Publisher(topic_root + "/mavros/rc/override", OverrideRCIn, queue_size=1)
        self.vision_pose_pub = rospy.Publisher(topic_root + "/mavros/vision_pose/pose", PoseStamped, queue_size=1)
        self.mocap_pose_pub = rospy.Publisher(topic_root + "/mavros/mocap/pose", PoseStamped, queue_size=1)
        self.nominal_pub = rospy.Publisher("/lpv_attack/nominal_setpoint", PoseStamped, queue_size=1)
        self.spoofed_pub = rospy.Publisher("/lpv_attack/spoofed_position", PoseStamped, queue_size=1)
        self.attacked_pub = rospy.Publisher("/lpv_attack/attacked_setpoint", PoseStamped, queue_size=1)
        self.attack_pub = rospy.Publisher("/lpv_attack/attack_signal", Vector3, queue_size=1)

        self.arm_srv = rospy.ServiceProxy(topic_root + "/mavros/cmd/arming", CommandBool)
        self.command_long_srv = rospy.ServiceProxy(topic_root + "/mavros/cmd/command", CommandLong)
        self.mode_srv = rospy.ServiceProxy(topic_root + "/mavros/set_mode", SetMode)
        self.param_set_srv = rospy.ServiceProxy(topic_root + "/mavros/param/set", ParamSet)

        self.state = State()
        self.estimated_pose_msg = None
        self.velocity = TwistStamped()
        self.actual_pose = None
        self.actual_velocity = None
        self.current_spoof = Vector3(0.0, 0.0, 0.0)
        self.last_spoofed_pose = None
        self.run_start_sim_time = None
        self.gps_takeoff_sim_time = None
        # Spawn point in the Gazebo world frame, captured at the first model-state
        # callback. MAVROS local position has its origin here, so to compare the
        # true (Gazebo) trajectory against the local-frame nominal/setpoint we log
        # actual = gazebo_world - origin.
        self.origin = None

    def _vector_param(self, name, default):
        data = rospy.get_param("~" + name, rospy.get_param(name, default))
        return Vector3(float(data.get("x", 0.0)), float(data.get("y", 0.0)), float(data.get("z", 0.0)))

    def _load_gps_spoof_sdf_params(self):
        if not self.gps_spoof_sdf or not os.path.exists(self.gps_spoof_sdf):
            rospy.logwarn("GPS spoof SDF not found; using ROS fallback attack params: %s", self.gps_spoof_sdf)
            return

        try:
            root = ET.parse(self.gps_spoof_sdf).getroot()
            plugin = root.find(".//plugin[@name='gps_plugin']")
            if plugin is None:
                rospy.logwarn("gps_plugin not found in %s; using ROS fallback attack params.", self.gps_spoof_sdf)
                return

            def get_float(name, default):
                elem = plugin.find(name)
                if elem is None or elem.text is None:
                    return default
                return float(elem.text)

            self.gps_spoof_start_s = get_float("gpsSpoofStart", self.gps_spoof_start_s)
            self.gps_spoof_end_s = get_float("gpsSpoofEnd", self.gps_spoof_end_s)
            self.gps_spoof_takeoff_z = get_float("gpsSpoofTakeoffZ", self.gps_spoof_takeoff_z)
            self.smooth_transition_duration = get_float("gpsSmoothTransitionDuration", self.smooth_transition_duration)
            self.smooth_exit_duration = get_float("gpsSmoothExitDuration", self.smooth_exit_duration)
            self.spoof_offset = Vector3(
                get_float("gpsSpoofOffsetX", self.spoof_offset.x),
                get_float("gpsSpoofOffsetY", self.spoof_offset.y),
                get_float("gpsSpoofOffsetZ", self.spoof_offset.z),
            )
            self.drift_rate = Vector3(
                get_float("gpsSpoofDriftX", self.drift_rate.x),
                get_float("gpsSpoofDriftY", self.drift_rate.y),
                get_float("gpsSpoofDriftZ", self.drift_rate.z),
            )
            self.attack_duration = max(0.0, self.gps_spoof_end_s - self.gps_spoof_start_s)
            rospy.loginfo(
                "Loaded GPS spoof params from SDF: start=%.2fs end=%.2fs offset=(%.3f, %.3f, %.3f) drift=(%.3f, %.3f, %.3f) smooth_in=%.2fs smooth_exit=%.2fs takeoff_z=%.2fm",
                self.gps_spoof_start_s, self.gps_spoof_end_s,
                self.spoof_offset.x, self.spoof_offset.y, self.spoof_offset.z,
                self.drift_rate.x, self.drift_rate.y, self.drift_rate.z,
                self.smooth_transition_duration, self.smooth_exit_duration, self.gps_spoof_takeoff_z,
            )
        except Exception as exc:
            rospy.logwarn("Failed to load GPS spoof params from %s: %s", self.gps_spoof_sdf, exc)

    def _state_cb(self, msg):
        self.state = msg

    def _pose_cb(self, msg):
        self.estimated_pose_msg = msg

    def _vel_cb(self, msg):
        self.velocity = msg

    def _model_states_cb(self, msg):
        for model_name in (self.vehicle_ns, "iris_0", "iris"):
            if model_name in msg.name:
                idx = msg.name.index(model_name)
                self.actual_pose = msg.pose[idx]
                self.actual_velocity = msg.twist[idx]
                if self.origin is None:
                    self.origin = copy.deepcopy(self.actual_pose.position)
                if self.gps_takeoff_sim_time is None and self.actual_pose.position.z > self.gps_spoof_takeoff_z:
                    now = rospy.Time.now().to_sec()
                    if now > 0.0:
                        self.gps_takeoff_sim_time = now
                        rospy.loginfo(
                            "GPS spoof takeoff reference detected at world_z=%.3fm, sim_t=%.3fs",
                            self.actual_pose.position.z,
                            self.gps_takeoff_sim_time,
                        )
                return

    def _make_pose(self, x, y, z, yaw):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        q = quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        return msg

    def _nominal_reference(self, t):
        if t < self.warmup_time:
            return self._make_pose(self.center_x, self.center_y, self.takeoff_height, self.yaw)

        tau = t - self.warmup_time
        if self.traj_type == "line":
            x = self.center_x + 0.08 * tau
            y = self.center_y
        else:
            theta = self.angular_rate * tau
            x = self.center_x + self.radius * (math.cos(theta) - 1.0)
            y = self.center_y + self.radius * math.sin(theta)
        return self._make_pose(x, y, self.takeoff_height, self.yaw)

    def _spoof_signal(self, t):
        window = self._attack_window(t)
        if window is None:
            return Vector3(0.0, 0.0, 0.0)
        attack_start, attack_end, attack_stop = window
        smooth_in_duration = max(0.0, self.smooth_transition_duration)
        smooth_exit_duration = max(0.0, self.smooth_exit_duration)
        if t < attack_start or t > attack_stop:
            return Vector3(0.0, 0.0, 0.0)

        tau = t - attack_start
        attack_duration = attack_end - attack_start

        # Smooth transition using cubic polynomial.
        # s(t) = 3t^2 - 2t^3 where t is in [0, 1].
        attack_scale = 1.0
        if smooth_in_duration > 0.0 and tau < smooth_in_duration:
            normalized_tau = tau / smooth_in_duration
            attack_scale = 3.0 * normalized_tau**2 - 2.0 * normalized_tau**3
        elif smooth_exit_duration > 0.0 and tau > attack_duration:
            time_since_end = tau - attack_duration
            normalized_tau = 1.0 - (time_since_end / smooth_exit_duration)
            normalized_tau = max(0.0, min(1.0, normalized_tau))
            attack_scale = 3.0 * normalized_tau**2 - 2.0 * normalized_tau**3

        # Freeze accumulated drift at attack end, then ramp the whole bias out.
        drift_tau = min(tau, attack_duration)
        return Vector3(
            (self.spoof_offset.x + self.drift_rate.x * drift_tau) * attack_scale,
            (self.spoof_offset.y + self.drift_rate.y * drift_tau) * attack_scale,
            (self.spoof_offset.z + self.drift_rate.z * drift_tau) * attack_scale,
        )

    def _attack_window(self, _t=None):
        if self.run_start_sim_time is None or self.gps_takeoff_sim_time is None:
            return None

        takeoff_t = self.gps_takeoff_sim_time - self.run_start_sim_time
        attack_start = takeoff_t + self.gps_spoof_start_s
        attack_end = takeoff_t + self.gps_spoof_end_s
        attack_stop = attack_end + max(0.0, self.smooth_exit_duration)
        return attack_start, attack_end, attack_stop

    def _phase_for_time(self, t):
        window = self._attack_window(t)
        if window is None:
            return "baseline"
        attack_start, attack_end, _attack_stop = window
        if t < attack_start:
            return "baseline"
        if t <= attack_end:
            return "attack"
        return "post_attack"

    def _make_raw_setpoint(self, pose_msg):
        raw = PositionTarget()
        raw.header.stamp = rospy.Time.now()
        raw.header.frame_id = "map"
        raw.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        raw.type_mask = (
            PositionTarget.IGNORE_VX
            + PositionTarget.IGNORE_VY
            + PositionTarget.IGNORE_VZ
            + PositionTarget.IGNORE_AFX
            + PositionTarget.IGNORE_AFY
            + PositionTarget.IGNORE_AFZ
            + PositionTarget.IGNORE_YAW_RATE
        )
        raw.position.x = pose_msg.pose.position.x
        raw.position.y = pose_msg.pose.position.y
        raw.position.z = pose_msg.pose.position.z
        raw.yaw = self.yaw
        return raw

    def _publish_setpoint(self, pose_msg):
        pose_msg.header.stamp = rospy.Time.now()
        self._publish_external_pose_estimate(self.current_spoof)
        self.setpoint_pub.publish(pose_msg)
        self.raw_setpoint_pub.publish(self._make_raw_setpoint(pose_msg))
        self._publish_neutral_manual_control()

    def _spoofed_pose_msg(self, spoof):
        if self.actual_pose is None:
            return None
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose = copy.deepcopy(self.actual_pose)
        pose.pose.position.x += spoof.x
        pose.pose.position.y += spoof.y
        pose.pose.position.z += spoof.z
        return pose

    def _publish_external_pose_estimate(self, spoof):
        # Disabled: the genuine GPS spoofing is now injected at the Gazebo GPS
        # sensor plugin (libgazebo_gps_plugin.so), which PX4's EKF2 actually
        # fuses. Publishing an external vision/mocap pose here was both ineffective
        # (PX4 navigates on GPS) and in the wrong frame, so it is intentionally
        # a no-op to keep PX4 on a pure-GPS estimate.
        return

    def _publish_neutral_manual_control(self):
        manual = ManualControl()
        manual.header.stamp = rospy.Time.now()
        manual.x = 0.0
        manual.y = 0.0
        manual.z = 0.5
        manual.r = 0.0
        manual.buttons = 0
        self.manual_pub.publish(manual)

        rc = OverrideRCIn()
        rc.channels = [
            1500, 1500, 1500, 1500,
            1500, 1500, 1500, 1500,
            OverrideRCIn.CHAN_NOCHANGE, OverrideRCIn.CHAN_NOCHANGE,
            OverrideRCIn.CHAN_NOCHANGE, OverrideRCIn.CHAN_NOCHANGE,
            OverrideRCIn.CHAN_NOCHANGE, OverrideRCIn.CHAN_NOCHANGE,
            OverrideRCIn.CHAN_NOCHANGE, OverrideRCIn.CHAN_NOCHANGE,
            OverrideRCIn.CHAN_NOCHANGE, OverrideRCIn.CHAN_NOCHANGE,
        ]
        self.rc_override_pub.publish(rc)

    def _wait_for_px4(self):
        rospy.loginfo("Waiting for MAVROS connection on /%s/mavros/state ...", self.vehicle_ns)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and not self.state.connected:
            rate.sleep()

        rospy.loginfo("Waiting for Gazebo model state ...")
        while not rospy.is_shutdown() and self.actual_pose is None:
            rate.sleep()

        rospy.loginfo("Checking MAVROS local position ...")
        pose_wait_start = time.monotonic()
        while not rospy.is_shutdown() and self.estimated_pose_msg is None and time.monotonic() - pose_wait_start < 12.0:
            self._publish_external_pose_estimate(Vector3(0.0, 0.0, 0.0))
            rate.sleep()
        if self.estimated_pose_msg is None:
            rospy.logwarn("MAVROS local position not received yet; continuing with Gazebo model state for logging.")

        rospy.wait_for_service("/" + self.vehicle_ns + "/mavros/cmd/arming")
        rospy.wait_for_service("/" + self.vehicle_ns + "/mavros/cmd/command")
        rospy.wait_for_service("/" + self.vehicle_ns + "/mavros/set_mode")
        rospy.wait_for_service("/" + self.vehicle_ns + "/mavros/param/set")
        self._configure_px4_for_offboard()

    def _set_px4_param(self, name, integer=0, real=0.0):
        try:
            value = ParamValue(integer=integer, real=real)
            result = self.param_set_srv(name, value)
            if result.success:
                rospy.loginfo("PX4 param set: %s = int:%s real:%s", name, integer, real)
            else:
                rospy.logwarn("PX4 param set failed: %s", name)
        except rospy.ServiceException as exc:
            rospy.logwarn("PX4 param set exception for %s: %s", name, exc)

    def _configure_px4_for_offboard(self):
        self._set_px4_param("COM_RC_IN_MODE", integer=4)
        self._set_px4_param("COM_RCL_EXCEPT", integer=4)
        self._set_px4_param("NAV_RCL_ACT", integer=0)

    def _enter_offboard(self, initial_setpoint):
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo("Streaming truthful localization and initial setpoints before OFFBOARD ...")
        self.current_spoof = Vector3(0.0, 0.0, 0.0)
        for _ in range(int(self.rate_hz * 3.0)):
            self._publish_setpoint(initial_setpoint)
            rate.sleep()

        last_mode_request = 0.0
        last_arm_request = 0.0
        while not rospy.is_shutdown():
            self._publish_setpoint(initial_setpoint)

            if self.state.mode != "OFFBOARD" and time.monotonic() - last_mode_request > 1.0:
                try:
                    if self.mode_srv(custom_mode="OFFBOARD").mode_sent:
                        rospy.loginfo("OFFBOARD mode requested.")
                except rospy.ServiceException as exc:
                    rospy.logwarn("OFFBOARD request failed: %s", exc)
                last_mode_request = time.monotonic()

            if self.state.mode == "OFFBOARD" and not self.state.armed and time.monotonic() - last_arm_request > 1.0:
                try:
                    if self.arm_srv(True).success:
                        rospy.loginfo("Vehicle armed.")
                except rospy.ServiceException as exc:
                    rospy.logwarn("Arming request failed: %s", exc)

                if not self.state.armed:
                    try:
                        result = self.command_long_srv(False, 400, 0, 1.0, 21196.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                        if result.success:
                            rospy.loginfo("Vehicle force-arm command accepted.")
                        else:
                            rospy.logwarn("Vehicle force-arm rejected with MAV_RESULT %s.", result.result)
                    except rospy.ServiceException as exc:
                        rospy.logwarn("Force-arm request failed: %s", exc)
                last_arm_request = time.monotonic()

            if self.state.mode == "OFFBOARD" and self.state.armed:
                return
            rate.sleep()

    def _pose_to_rpy(self, pose):
        if pose is None:
            return 0.0, 0.0, 0.0
        q = pose.orientation
        return euler_from_quaternion([q.x, q.y, q.z, q.w])

    def _dist(self, p, q):
        return math.sqrt((p.x - q.x) ** 2 + (p.y - q.y) ** 2 + (p.z - q.z) ** 2)

    def _dist_xyz(self, x1, y1, z1, x2, y2, z2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

    def _write_header(self, writer):
        writer.writerow([
            "t", "phase",
            "nominal_x", "nominal_y", "nominal_z",
            "attacked_x", "attacked_y", "attacked_z",
            "attack_x", "attack_y", "attack_z",
            "actual_x", "actual_y", "actual_z",
            "estimated_x", "estimated_y", "estimated_z",
            "spoofed_x", "spoofed_y", "spoofed_z",
            "vel_x", "vel_y", "vel_z",
            "roll", "pitch", "yaw",
            "err_to_nominal", "err_to_attacked",
            "estimated_err_to_nominal", "estimate_actual_gap",
            "mavros_mode", "armed",
        ])

    def _log_row(self, writer, t, phase, nominal, spoof):
        if self.actual_pose is None or self.origin is None:
            return
        # Convert actual (Gazebo world) to local frame (origin at spawn)
        actual_local_x = self.actual_pose.position.x - self.origin.x
        actual_local_y = self.actual_pose.position.y - self.origin.y
        actual_local_z = self.actual_pose.position.z - self.origin.z
        estimated = self.estimated_pose_msg.pose.position if self.estimated_pose_msg is not None else self.actual_pose.position
        spoofed_msg = self.last_spoofed_pose or self._spoofed_pose_msg(spoof)
        spoofed = spoofed_msg.pose.position if spoofed_msg is not None else self.actual_pose.position
        v = self.actual_velocity.linear if self.actual_velocity is not None else self.velocity.twist.linear
        roll, pitch, yaw = self._pose_to_rpy(self.actual_pose)

        writer.writerow([
            "%.4f" % t, phase,
            "%.6f" % nominal.pose.position.x, "%.6f" % nominal.pose.position.y, "%.6f" % nominal.pose.position.z,
            "%.6f" % spoofed.x, "%.6f" % spoofed.y, "%.6f" % spoofed.z,
            "%.6f" % spoof.x, "%.6f" % spoof.y, "%.6f" % spoof.z,
            "%.6f" % actual_local_x, "%.6f" % actual_local_y, "%.6f" % actual_local_z,
            "%.6f" % estimated.x, "%.6f" % estimated.y, "%.6f" % estimated.z,
            "%.6f" % spoofed.x, "%.6f" % spoofed.y, "%.6f" % spoofed.z,
            "%.6f" % v.x, "%.6f" % v.y, "%.6f" % v.z,
            "%.6f" % roll, "%.6f" % pitch, "%.6f" % yaw,
            "%.6f" % self._dist_xyz(actual_local_x, actual_local_y, actual_local_z, nominal.pose.position.x, nominal.pose.position.y, nominal.pose.position.z),
            "%.6f" % self._dist_xyz(actual_local_x, actual_local_y, actual_local_z, spoofed.x, spoofed.y, spoofed.z),
            "%.6f" % self._dist(estimated, nominal.pose.position),
            "%.6f" % self._dist_xyz(estimated.x, estimated.y, estimated.z, actual_local_x, actual_local_y, actual_local_z),
            self.state.mode, str(self.state.armed),
        ])

    def run(self):
        self._wait_for_px4()
        initial = self._make_pose(self.center_x, self.center_y, self.takeoff_height, self.yaw)
        self._enter_offboard(initial)

        fallback_total_time = max(
            self.warmup_time + self.pre_attack_time + self.attack_duration + self.post_attack_time,
            self.gps_spoof_end_s + self.smooth_exit_duration + self.post_attack_time + 8.0,
        )

        rospy.loginfo("GPS spoofing attack demo started. CSV: %s", self.csv_path)
        rospy.loginfo(
            "Attack window: %.1fs to %.1fs after GPS takeoff reference (world_z > %.2fm)",
            self.gps_spoof_start_s,
            self.gps_spoof_end_s,
            self.gps_spoof_takeoff_z,
        )

        rate = rospy.Rate(self.rate_hz)
        self.run_start_sim_time = rospy.Time.now().to_sec()
        start_wall = time.monotonic()
        last_mode_request = time.monotonic()

        with open(self.csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            self._write_header(writer)

            while not rospy.is_shutdown():
                now_sim = rospy.Time.now().to_sec()
                if self.run_start_sim_time > 0.0 and now_sim > 0.0:
                    t = now_sim - self.run_start_sim_time
                else:
                    t = time.monotonic() - start_wall

                window = self._attack_window(t)
                if window is not None:
                    _attack_start, _attack_end, attack_stop = window
                    if t > attack_stop + self.post_attack_time:
                        break
                elif t > fallback_total_time:
                    break

                nominal = self._nominal_reference(t)
                spoof = self._spoof_signal(t)
                self.current_spoof = spoof

                phase = self._phase_for_time(t)

                self._publish_setpoint(nominal)
                spoofed_msg = self.last_spoofed_pose or self._spoofed_pose_msg(spoof)
                self.nominal_pub.publish(nominal)
                if spoofed_msg is not None:
                    self.spoofed_pub.publish(spoofed_msg)
                    self.attacked_pub.publish(spoofed_msg)
                self.attack_pub.publish(spoof)
                self._log_row(writer, t, phase, nominal, spoof)

                if self.state.mode != "OFFBOARD" and time.monotonic() - last_mode_request > 5.0:
                    try:
                        self.mode_srv(custom_mode="OFFBOARD")
                    except rospy.ServiceException:
                        pass
                    last_mode_request = time.monotonic()

                rate.sleep()

        rospy.loginfo("GPS spoofing attack demo finished. Result saved to %s", self.csv_path)


if __name__ == "__main__":
    rospy.init_node("gps_spoof_attack_demo")
    GpsSpoofAttackDemo().run()
