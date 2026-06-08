#!/usr/bin/env python3
import csv
import math
import os
import sys
import time

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


class SetpointFdiAttackDemo:
    def __init__(self):
        self.vehicle_ns = rospy.get_param("~vehicle_ns", rospy.get_param("vehicle_ns", "iris_0")).strip("/")
        self.rate_hz = float(rospy.get_param("~rate_hz", rospy.get_param("rate_hz", 30.0)))

        self.takeoff_height = float(rospy.get_param("~takeoff_height", rospy.get_param("takeoff_height", 2.0)))
        self.warmup_time = float(rospy.get_param("~warmup_time", rospy.get_param("warmup_time", 4.0)))
        self.pre_attack_time = float(rospy.get_param("~pre_attack_time", rospy.get_param("pre_attack_time", 18.0)))
        self.attack_duration = float(rospy.get_param("~attack_duration", rospy.get_param("attack_duration", 35.0)))
        self.post_attack_time = float(rospy.get_param("~post_attack_time", rospy.get_param("post_attack_time", 12.0)))

        self.attack_offset = self._vector_param("attack_offset", {"x": 1.2, "y": 0.8, "z": 0.0})
        self.drift_rate = self._vector_param("drift_rate", {"x": 0.015, "y": 0.0, "z": 0.0})

        self.traj_type = rospy.get_param("~trajectory/type", rospy.get_param("trajectory/type", "circle"))
        self.center_x = float(rospy.get_param("~trajectory/center_x", rospy.get_param("trajectory/center_x", 0.0)))
        self.center_y = float(rospy.get_param("~trajectory/center_y", rospy.get_param("trajectory/center_y", 0.0)))
        self.radius = float(rospy.get_param("~trajectory/radius", rospy.get_param("trajectory/radius", 2.0)))
        self.angular_rate = float(rospy.get_param("~trajectory/angular_rate", rospy.get_param("trajectory/angular_rate", 0.12)))
        self.yaw = float(rospy.get_param("~trajectory/yaw", rospy.get_param("trajectory/yaw", 0.0)))

        self.results_dir = rospy.get_param("~results_dir", rospy.get_param("results_dir", "/tmp"))
        self.run_dir, self.run_stamp = make_run_dir(self.results_dir)
        self.csv_path = os.path.join(self.run_dir, "setpoint_fdi_attack_%s.csv" % self.run_stamp)
        rospy.loginfo("Setpoint FDI results directory: %s", self.run_dir)

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
        self.attacked_pub = rospy.Publisher("/lpv_attack/attacked_setpoint", PoseStamped, queue_size=1)
        self.attack_pub = rospy.Publisher("/lpv_attack/attack_signal", Vector3, queue_size=1)

        self.arm_srv = rospy.ServiceProxy(topic_root + "/mavros/cmd/arming", CommandBool)
        self.command_long_srv = rospy.ServiceProxy(topic_root + "/mavros/cmd/command", CommandLong)
        self.mode_srv = rospy.ServiceProxy(topic_root + "/mavros/set_mode", SetMode)
        self.param_set_srv = rospy.ServiceProxy(topic_root + "/mavros/param/set", ParamSet)

        self.state = State()
        self.pose = None
        self.velocity = TwistStamped()
        self.actual_pose = None
        self.actual_velocity = None

    def _vector_param(self, name, default):
        data = rospy.get_param("~" + name, rospy.get_param(name, default))
        return Vector3(float(data.get("x", 0.0)), float(data.get("y", 0.0)), float(data.get("z", 0.0)))

    def _state_cb(self, msg):
        self.state = msg

    def _pose_cb(self, msg):
        self.pose = msg
        self.actual_pose = msg.pose

    def _vel_cb(self, msg):
        self.velocity = msg
        self.actual_velocity = msg.twist

    def _model_states_cb(self, msg):
        for model_name in (self.vehicle_ns, "iris_0", "iris"):
            if model_name in msg.name:
                idx = msg.name.index(model_name)
                self.actual_pose = msg.pose[idx]
                self.actual_velocity = msg.twist[idx]
                self._publish_external_pose_estimate()
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

    def _attack_signal(self, t):
        attack_start = self.warmup_time + self.pre_attack_time
        attack_end = attack_start + self.attack_duration
        if t < attack_start or t > attack_end:
            return Vector3(0.0, 0.0, 0.0)
        tau = t - attack_start
        return Vector3(
            self.attack_offset.x + self.drift_rate.x * tau,
            self.attack_offset.y + self.drift_rate.y * tau,
            self.attack_offset.z + self.drift_rate.z * tau,
        )

    def _apply_attack(self, nominal, attack):
        attacked = self._make_pose(
            nominal.pose.position.x + attack.x,
            nominal.pose.position.y + attack.y,
            nominal.pose.position.z + attack.z,
            self.yaw,
        )
        return attacked

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
        self._publish_external_pose_estimate()
        self.setpoint_pub.publish(pose_msg)
        self.raw_setpoint_pub.publish(self._make_raw_setpoint(pose_msg))
        self._publish_neutral_manual_control()

    def _publish_external_pose_estimate(self):
        if self.actual_pose is None:
            return
        if not hasattr(self, "vision_pose_pub") or not hasattr(self, "mocap_pose_pub"):
            return

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose = self.actual_pose
        try:
            self.vision_pose_pub.publish(pose)
            self.mocap_pose_pub.publish(pose)
        except rospy.ROSException:
            pass

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
        while not rospy.is_shutdown() and self.pose is None and time.monotonic() - pose_wait_start < 12.0:
            self._publish_external_pose_estimate()
            rate.sleep()
        if self.pose is None:
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
        # Required in this SITL setup: otherwise PX4 refuses arming with
        # "manual control lost" when no RC transmitter is present.
        self._set_px4_param("COM_RC_IN_MODE", integer=4)
        self._set_px4_param("COM_RCL_EXCEPT", integer=4)
        self._set_px4_param("NAV_RCL_ACT", integer=0)

    def _enter_offboard(self, initial_setpoint):
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo("Streaming initial setpoints before OFFBOARD ...")
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
                        # PX4 SITL can still reject normal arming in this
                        # workspace with "manual control lost". Param2=21196
                        # is PX4's documented force-arm value for simulation
                        # and bench testing through MAV_CMD_COMPONENT_ARM_DISARM.
                        result = self.command_long_srv(
                            False,
                            400,
                            0,
                            1.0,
                            21196.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                        )
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

    def _pose_to_rpy(self):
        if self.actual_pose is None:
            return 0.0, 0.0, 0.0
        q = self.actual_pose.orientation
        return euler_from_quaternion([q.x, q.y, q.z, q.w])

    def _write_header(self, writer):
        writer.writerow([
            "t",
            "phase",
            "nominal_x", "nominal_y", "nominal_z",
            "attacked_x", "attacked_y", "attacked_z",
            "attack_x", "attack_y", "attack_z",
            "actual_x", "actual_y", "actual_z",
            "vel_x", "vel_y", "vel_z",
            "roll", "pitch", "yaw",
            "err_to_nominal", "err_to_attacked",
            "mavros_mode", "armed",
        ])

    def _log_row(self, writer, t, phase, nominal, attacked, attack):
        if self.actual_pose is None:
            return
        p = self.actual_pose.position
        v = self.actual_velocity.linear if self.actual_velocity is not None else self.velocity.twist.linear
        roll, pitch, yaw = self._pose_to_rpy()
        err_nominal = math.sqrt(
            (p.x - nominal.pose.position.x) ** 2
            + (p.y - nominal.pose.position.y) ** 2
            + (p.z - nominal.pose.position.z) ** 2
        )
        err_attacked = math.sqrt(
            (p.x - attacked.pose.position.x) ** 2
            + (p.y - attacked.pose.position.y) ** 2
            + (p.z - attacked.pose.position.z) ** 2
        )
        writer.writerow([
            "%.4f" % t,
            phase,
            "%.6f" % nominal.pose.position.x, "%.6f" % nominal.pose.position.y, "%.6f" % nominal.pose.position.z,
            "%.6f" % attacked.pose.position.x, "%.6f" % attacked.pose.position.y, "%.6f" % attacked.pose.position.z,
            "%.6f" % attack.x, "%.6f" % attack.y, "%.6f" % attack.z,
            "%.6f" % p.x, "%.6f" % p.y, "%.6f" % p.z,
            "%.6f" % v.x, "%.6f" % v.y, "%.6f" % v.z,
            "%.6f" % roll, "%.6f" % pitch, "%.6f" % yaw,
            "%.6f" % err_nominal, "%.6f" % err_attacked,
            self.state.mode, str(self.state.armed),
        ])

    def run(self):
        self._wait_for_px4()
        initial = self._make_pose(self.center_x, self.center_y, self.takeoff_height, self.yaw)
        self._enter_offboard(initial)

        total_time = self.warmup_time + self.pre_attack_time + self.attack_duration + self.post_attack_time
        attack_start = self.warmup_time + self.pre_attack_time
        attack_end = attack_start + self.attack_duration

        rospy.loginfo("FDI attack demo started. CSV: %s", self.csv_path)
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
                attack = self._attack_signal(t)
                attacked = self._apply_attack(nominal, attack)

                if t < attack_start:
                    phase = "baseline"
                elif t <= attack_end:
                    phase = "attack"
                else:
                    phase = "post_attack"

                self._publish_setpoint(attacked)
                self.nominal_pub.publish(nominal)
                self.attacked_pub.publish(attacked)
                self.attack_pub.publish(attack)
                self._log_row(writer, t, phase, nominal, attacked, attack)

                if self.state.mode != "OFFBOARD" and time.monotonic() - last_mode_request > 5.0:
                    try:
                        self.mode_srv(custom_mode="OFFBOARD")
                    except rospy.ServiceException:
                        pass
                    last_mode_request = time.monotonic()

                rate.sleep()

        rospy.loginfo("FDI attack demo finished. Result saved to %s", self.csv_path)


if __name__ == "__main__":
    rospy.init_node("setpoint_fdi_attack_demo")
    SetpointFdiAttackDemo().run()
