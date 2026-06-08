#!/usr/bin/env python3
import csv
import math
import os
import time
from datetime import datetime

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, Vector3, TwistStamped
from mavros_msgs.msg import GPSINPUT, HilGPS, ManualControl, OverrideRCIn, ParamValue, PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandLong, ParamSet, SetMode
from sensor_msgs.msg import NavSatFix
from tf.transformations import euler_from_quaternion, quaternion_from_euler


GPS_EPOCH_UNIX = 315964800.0
EARTH_RADIUS_M = 6378137.0


class TrueGpsInputAttackDemo:
    def __init__(self):
        self.vehicle_ns = rospy.get_param("~vehicle_ns", rospy.get_param("vehicle_ns", "iris_0")).strip("/")
        self.rate_hz = float(rospy.get_param("~rate_hz", rospy.get_param("rate_hz", 30.0)))
        self.gps_rate_hz = float(rospy.get_param("~gps_rate_hz", rospy.get_param("gps_rate_hz", 15.0)))
        self.publish_external_gps = bool(rospy.get_param("~publish_external_gps", rospy.get_param("publish_external_gps", True)))

        self.takeoff_height = float(rospy.get_param("~takeoff_height", rospy.get_param("takeoff_height", 2.0)))
        self.warmup_time = float(rospy.get_param("~warmup_time", rospy.get_param("warmup_time", 4.0)))
        self.pre_attack_time = float(rospy.get_param("~pre_attack_time", rospy.get_param("pre_attack_time", 18.0)))
        self.attack_duration = float(rospy.get_param("~attack_duration", rospy.get_param("attack_duration", 35.0)))
        self.post_attack_time = float(rospy.get_param("~post_attack_time", rospy.get_param("post_attack_time", 12.0)))

        self.gps_origin = rospy.get_param("~gps_origin", rospy.get_param("gps_origin", {
            "lat": 47.3667, "lon": 8.55, "alt": 408.0,
        }))
        self.origin_lat = float(self.gps_origin.get("lat", 47.3667))
        self.origin_lon = float(self.gps_origin.get("lon", 8.55))
        self.origin_alt = float(self.gps_origin.get("alt", 408.0))

        self.spoof_offset = self._vector_param("gps_spoof_offset", {"x": 1.2, "y": 0.8, "z": 0.0})
        self.drift_rate = self._vector_param("gps_spoof_drift_rate", {"x": 0.015, "y": 0.0, "z": 0.0})

        self.traj_type = rospy.get_param("~trajectory/type", rospy.get_param("trajectory/type", "circle"))
        self.center_x = float(rospy.get_param("~trajectory/center_x", rospy.get_param("trajectory/center_x", 0.0)))
        self.center_y = float(rospy.get_param("~trajectory/center_y", rospy.get_param("trajectory/center_y", 0.0)))
        self.radius = float(rospy.get_param("~trajectory/radius", rospy.get_param("trajectory/radius", 2.0)))
        self.angular_rate = float(rospy.get_param("~trajectory/angular_rate", rospy.get_param("trajectory/angular_rate", 0.12)))
        self.yaw = float(rospy.get_param("~trajectory/yaw", rospy.get_param("trajectory/yaw", 0.0)))

        self.results_dir = rospy.get_param("~results_dir", rospy.get_param("results_dir", "/tmp"))
        os.makedirs(self.results_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.results_dir, "true_gps_input_attack_%s.csv" % stamp)

        topic_root = "/" + self.vehicle_ns
        self.state_sub = rospy.Subscriber(topic_root + "/mavros/state", State, self._state_cb, queue_size=1)
        self.pose_sub = rospy.Subscriber(topic_root + "/mavros/local_position/pose", PoseStamped, self._pose_cb, queue_size=1)
        self.vel_sub = rospy.Subscriber(topic_root + "/mavros/local_position/velocity_local", TwistStamped, self._vel_cb, queue_size=1)
        self.global_sub = rospy.Subscriber(topic_root + "/mavros/global_position/global", NavSatFix, self._global_cb, queue_size=1)
        self.raw_gps_sub = rospy.Subscriber(topic_root + "/mavros/global_position/raw/fix", NavSatFix, self._raw_gps_cb, queue_size=1)
        self.model_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states_cb, queue_size=1)

        self.hil_gps_pub = rospy.Publisher(topic_root + "/mavros/hil/gps", HilGPS, queue_size=1)
        self.gps_input_pub = rospy.Publisher(topic_root + "/mavros/gps_input/gps_input", GPSINPUT, queue_size=1)
        self.setpoint_pub = rospy.Publisher(topic_root + "/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.raw_setpoint_pub = rospy.Publisher(topic_root + "/mavros/setpoint_raw/local", PositionTarget, queue_size=1)
        self.manual_pub = rospy.Publisher(topic_root + "/mavros/manual_control/send", ManualControl, queue_size=1)
        self.rc_override_pub = rospy.Publisher(topic_root + "/mavros/rc/override", OverrideRCIn, queue_size=1)
        self.nominal_pub = rospy.Publisher("/lpv_attack/nominal_setpoint", PoseStamped, queue_size=1)
        self.gps_spoofed_pub = rospy.Publisher("/lpv_attack/spoofed_gps_local", PoseStamped, queue_size=1)
        self.attack_pub = rospy.Publisher("/lpv_attack/gps_attack_signal", Vector3, queue_size=1)

        self.arm_srv = rospy.ServiceProxy(topic_root + "/mavros/cmd/arming", CommandBool)
        self.command_long_srv = rospy.ServiceProxy(topic_root + "/mavros/cmd/command", CommandLong)
        self.mode_srv = rospy.ServiceProxy(topic_root + "/mavros/set_mode", SetMode)
        self.param_set_srv = rospy.ServiceProxy(topic_root + "/mavros/param/set", ParamSet)

        self.state = State()
        self.estimated_pose_msg = None
        self.velocity = TwistStamped()
        self.global_fix = None
        self.raw_gps_fix = None
        self.actual_pose = None
        self.actual_velocity = None
        self.current_spoof = Vector3(0.0, 0.0, 0.0)
        self.last_gps_local = None
        self.last_gps_lat = float("nan")
        self.last_gps_lon = float("nan")
        self.last_gps_alt = float("nan")
        self.last_hil_gps_time = 0.0

    def _vector_param(self, name, default):
        data = rospy.get_param("~" + name, rospy.get_param(name, default))
        return Vector3(float(data.get("x", 0.0)), float(data.get("y", 0.0)), float(data.get("z", 0.0)))

    def _state_cb(self, msg):
        self.state = msg

    def _pose_cb(self, msg):
        self.estimated_pose_msg = msg

    def _vel_cb(self, msg):
        self.velocity = msg

    def _global_cb(self, msg):
        self.global_fix = msg

    def _raw_gps_cb(self, msg):
        self.raw_gps_fix = msg

    def _model_states_cb(self, msg):
        for model_name in (self.vehicle_ns, "iris_0", "iris"):
            if model_name in msg.name:
                idx = msg.name.index(model_name)
                self.actual_pose = msg.pose[idx]
                self.actual_velocity = msg.twist[idx]
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
        attack_start = self.warmup_time + self.pre_attack_time
        attack_end = attack_start + self.attack_duration
        if t < attack_start or t > attack_end:
            return Vector3(0.0, 0.0, 0.0)
        tau = t - attack_start
        return Vector3(
            self.spoof_offset.x + self.drift_rate.x * tau,
            self.spoof_offset.y + self.drift_rate.y * tau,
            self.spoof_offset.z + self.drift_rate.z * tau,
        )

    def _local_to_wgs84(self, north_m, east_m, up_m):
        lat = self.origin_lat + math.degrees(north_m / EARTH_RADIUS_M)
        lon = self.origin_lon + math.degrees(east_m / (EARTH_RADIUS_M * math.cos(math.radians(self.origin_lat))))
        alt = self.origin_alt + up_m
        return lat, lon, alt

    def _gps_week(self, stamp):
        gps_time = max(0.0, stamp.to_sec() - GPS_EPOCH_UNIX)
        week = int(gps_time // 604800.0)
        week_ms = int((gps_time - week * 604800.0) * 1000.0) % 604800000
        return week, week_ms

    def _gps_local_pose(self, spoof):
        if self.actual_pose is None:
            return None
        p = self.actual_pose.position
        return self._make_pose(p.x + spoof.x, p.y + spoof.y, p.z + spoof.z, self.yaw)

    def _publish_gps_measurement(self, spoof, force=False):
        if not self.publish_external_gps:
            gps_local = self._gps_local_pose(spoof)
            if gps_local is not None:
                p = gps_local.pose.position
                lat, lon, alt = self._local_to_wgs84(p.y, p.x, p.z)
                self.gps_spoofed_pub.publish(gps_local)
                self.last_gps_local = gps_local
                self.last_gps_lat = lat
                self.last_gps_lon = lon
                self.last_gps_alt = alt
            return

        now_mono = time.monotonic()
        if not force and now_mono - self.last_hil_gps_time < 1.0 / max(self.gps_rate_hz, 1.0):
            return
        if self.actual_pose is None:
            return

        gps_local = self._gps_local_pose(spoof)
        p = gps_local.pose.position
        # MAVROS local setpoints are ENU-like: x is east, y is north, z is up.
        # GPS latitude follows north and longitude follows east.
        lat, lon, alt = self._local_to_wgs84(p.y, p.x, p.z)
        v = self.actual_velocity.linear if self.actual_velocity is not None else self.velocity.twist.linear
        vn = v.y
        ve = v.x
        vd = -v.z
        ground_speed = math.sqrt(vn * vn + ve * ve)
        speed_3d = math.sqrt(vn * vn + ve * ve + vd * vd)
        cog = 65535 if ground_speed < 0.05 else int(round(math.degrees(math.atan2(ve, vn)) * 100.0)) % 36000

        stamp = rospy.Time.now()
        hil = HilGPS()
        hil.header.stamp = stamp
        hil.header.frame_id = "gps"
        hil.fix_type = 3
        hil.geo.latitude = lat
        hil.geo.longitude = lon
        hil.geo.altitude = alt
        hil.eph = 100
        hil.epv = 150
        hil.vel = min(65535, int(round(speed_3d * 100.0)))
        hil.vn = max(-32768, min(32767, int(round(vn * 100.0))))
        hil.ve = max(-32768, min(32767, int(round(ve * 100.0))))
        hil.vd = max(-32768, min(32767, int(round(vd * 100.0))))
        hil.cog = cog
        hil.satellites_visible = 10

        week, week_ms = self._gps_week(stamp)
        gps_input = GPSINPUT()
        gps_input.header.stamp = stamp
        gps_input.header.frame_id = "gps"
        gps_input.fix_type = GPSINPUT.GPS_FIX_TYPE_3D_FIX
        gps_input.gps_id = 0
        gps_input.ignore_flags = 0
        gps_input.time_week = week
        gps_input.time_week_ms = week_ms
        gps_input.lat = int(round(lat * 1e7))
        gps_input.lon = int(round(lon * 1e7))
        gps_input.alt = alt
        gps_input.hdop = 1.0
        gps_input.vdop = 1.5
        gps_input.vn = vn
        gps_input.ve = ve
        gps_input.vd = vd
        gps_input.speed_accuracy = 0.2
        gps_input.horiz_accuracy = 1.0
        gps_input.vert_accuracy = 1.5
        gps_input.satellites_visible = 10
        gps_input.yaw = 0

        self.hil_gps_pub.publish(hil)
        self.gps_input_pub.publish(gps_input)
        self.gps_spoofed_pub.publish(gps_local)
        self.last_gps_local = gps_local
        self.last_gps_lat = lat
        self.last_gps_lon = lon
        self.last_gps_alt = alt
        self.last_hil_gps_time = now_mono

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
        self._publish_gps_measurement(self.current_spoof)
        self.setpoint_pub.publish(pose_msg)
        self.raw_setpoint_pub.publish(self._make_raw_setpoint(pose_msg))
        self._publish_neutral_manual_control()

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
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.state.connected:
            rate.sleep()

        rospy.loginfo("Waiting for Gazebo model state ...")
        while not rospy.is_shutdown() and self.actual_pose is None:
            rate.sleep()

        rospy.wait_for_service("/" + self.vehicle_ns + "/mavros/param/set")
        self._configure_px4_for_gps_offboard()

        if self.publish_external_gps:
            rospy.loginfo("Streaming synthetic GPS through MAVROS HIL_GPS before EKF initialization ...")
            init_start = time.monotonic()
            while not rospy.is_shutdown() and time.monotonic() - init_start < 8.0:
                self.current_spoof = Vector3(0.0, 0.0, 0.0)
                self._publish_gps_measurement(self.current_spoof, force=True)
                rate.sleep()

        rospy.loginfo("Checking MAVROS local/global position from spoofable GPS input ...")
        pose_wait_start = time.monotonic()
        while not rospy.is_shutdown() and self.estimated_pose_msg is None and time.monotonic() - pose_wait_start < 18.0:
            if self.publish_external_gps:
                self._publish_gps_measurement(Vector3(0.0, 0.0, 0.0), force=True)
            rate.sleep()
        if self.estimated_pose_msg is None:
            rospy.logwarn("MAVROS local position not received yet; OFFBOARD may be rejected until EKF accepts GPS.")

        rospy.wait_for_service("/" + self.vehicle_ns + "/mavros/cmd/arming")
        rospy.wait_for_service("/" + self.vehicle_ns + "/mavros/cmd/command")
        rospy.wait_for_service("/" + self.vehicle_ns + "/mavros/set_mode")

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

    def _configure_px4_for_gps_offboard(self):
        self._set_px4_param("MAV_USEHILGPS", integer=1 if self.publish_external_gps else 0)
        self._set_px4_param("EKF2_AID_MASK", integer=1)
        self._set_px4_param("EKF2_HGT_MODE", integer=0)
        self._set_px4_param("EKF2_GPS_CHECK", integer=0)
        self._set_px4_param("SENS_GPS_PRIME", integer=0)
        self._set_px4_param("COM_RC_IN_MODE", integer=4)
        self._set_px4_param("COM_RCL_EXCEPT", integer=4)
        self._set_px4_param("NAV_RCL_ACT", integer=0)

    def _enter_offboard(self, initial_setpoint):
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo("Streaming GPS measurements and initial setpoints before OFFBOARD ...")
        self.current_spoof = Vector3(0.0, 0.0, 0.0)
        for _ in range(int(self.rate_hz * 4.0)):
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

    def _write_header(self, writer):
        writer.writerow([
            "t", "phase",
            "nominal_x", "nominal_y", "nominal_z",
            "gps_spoofed_x", "gps_spoofed_y", "gps_spoofed_z",
            "attack_x", "attack_y", "attack_z",
            "actual_x", "actual_y", "actual_z",
            "estimated_x", "estimated_y", "estimated_z",
            "vel_x", "vel_y", "vel_z",
            "roll", "pitch", "yaw",
            "gps_lat", "gps_lon", "gps_alt",
            "global_lat", "global_lon", "global_alt",
            "raw_gps_lat", "raw_gps_lon", "raw_gps_alt",
            "err_actual_to_nominal", "err_actual_to_gps_spoofed",
            "err_estimated_to_nominal", "estimate_actual_gap",
            "mavros_mode", "armed",
        ])

    def _fix_values(self, fix):
        if fix is None:
            return float("nan"), float("nan"), float("nan")
        return fix.latitude, fix.longitude, fix.altitude

    def _log_row(self, writer, t, phase, nominal, spoof):
        if self.actual_pose is None:
            return
        actual = self.actual_pose.position
        estimated = self.estimated_pose_msg.pose.position if self.estimated_pose_msg is not None else actual
        gps_local_msg = self.last_gps_local or self._gps_local_pose(spoof)
        gps_local = gps_local_msg.pose.position if gps_local_msg is not None else actual
        v = self.actual_velocity.linear if self.actual_velocity is not None else self.velocity.twist.linear
        roll, pitch, yaw = self._pose_to_rpy(self.actual_pose)
        global_lat, global_lon, global_alt = self._fix_values(self.global_fix)
        raw_lat, raw_lon, raw_alt = self._fix_values(self.raw_gps_fix)

        writer.writerow([
            "%.4f" % t, phase,
            "%.6f" % nominal.pose.position.x, "%.6f" % nominal.pose.position.y, "%.6f" % nominal.pose.position.z,
            "%.6f" % gps_local.x, "%.6f" % gps_local.y, "%.6f" % gps_local.z,
            "%.6f" % spoof.x, "%.6f" % spoof.y, "%.6f" % spoof.z,
            "%.6f" % actual.x, "%.6f" % actual.y, "%.6f" % actual.z,
            "%.6f" % estimated.x, "%.6f" % estimated.y, "%.6f" % estimated.z,
            "%.6f" % v.x, "%.6f" % v.y, "%.6f" % v.z,
            "%.6f" % roll, "%.6f" % pitch, "%.6f" % yaw,
            "%.8f" % self.last_gps_lat, "%.8f" % self.last_gps_lon, "%.3f" % self.last_gps_alt,
            "%.8f" % global_lat, "%.8f" % global_lon, "%.3f" % global_alt,
            "%.8f" % raw_lat, "%.8f" % raw_lon, "%.3f" % raw_alt,
            "%.6f" % self._dist(actual, nominal.pose.position),
            "%.6f" % self._dist(actual, gps_local),
            "%.6f" % self._dist(estimated, nominal.pose.position),
            "%.6f" % self._dist(estimated, actual),
            self.state.mode, str(self.state.armed),
        ])

    def run(self):
        self._wait_for_px4()
        initial = self._make_pose(self.center_x, self.center_y, self.takeoff_height, self.yaw)
        self._enter_offboard(initial)

        total_time = self.warmup_time + self.pre_attack_time + self.attack_duration + self.post_attack_time
        attack_start = self.warmup_time + self.pre_attack_time
        attack_end = attack_start + self.attack_duration

        rospy.loginfo("True GPS input spoofing demo started. CSV: %s", self.csv_path)
        rospy.loginfo("Attack window: %.1fs to %.1fs", attack_start, attack_end)

        rate = rospy.Rate(self.rate_hz)
        start = time.monotonic()
        last_mode_request = time.monotonic()

        with open(self.csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            self._write_header(writer)

            while not rospy.is_shutdown():
                t = time.monotonic() - start
                if t > total_time:
                    break

                nominal = self._nominal_reference(t)
                spoof = self._spoof_signal(t)
                self.current_spoof = spoof

                if t < attack_start:
                    phase = "baseline"
                elif t <= attack_end:
                    phase = "attack"
                else:
                    phase = "post_attack"

                self._publish_setpoint(nominal)
                self.nominal_pub.publish(nominal)
                self.attack_pub.publish(spoof)
                self._log_row(writer, t, phase, nominal, spoof)

                if self.state.mode != "OFFBOARD" and time.monotonic() - last_mode_request > 5.0:
                    try:
                        self.mode_srv(custom_mode="OFFBOARD")
                    except rospy.ServiceException:
                        pass
                    last_mode_request = time.monotonic()

                rate.sleep()

        rospy.loginfo("True GPS input spoofing demo finished. Result saved to %s", self.csv_path)


if __name__ == "__main__":
    rospy.init_node("true_gps_input_attack_demo")
    TrueGpsInputAttackDemo().run()
