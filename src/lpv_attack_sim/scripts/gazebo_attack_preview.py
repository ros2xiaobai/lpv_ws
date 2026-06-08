#!/usr/bin/env python3
import math
import time

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, SetModelState, SpawnModel
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA
from tf.transformations import quaternion_from_euler


def rgba(r, g, b, a=1.0):
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = r, g, b, a
    return c


class GazeboAttackPreview:
    def __init__(self):
        self.rate_hz = 30.0
        self.takeoff_height = 2.0
        self.warmup_time = 4.0
        self.pre_attack_time = 18.0
        self.attack_duration = 35.0
        self.post_attack_time = 12.0
        self.total_time = self.warmup_time + self.pre_attack_time + self.attack_duration + self.post_attack_time
        self.attack_start = self.warmup_time + self.pre_attack_time
        self.attack_end = self.attack_start + self.attack_duration
        self.radius = 2.0
        self.angular_rate = 0.12
        self.actual = [0.0, 0.0, 0.0]

        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/delete_model")
        rospy.wait_for_service("/gazebo/set_model_state")
        self.spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.delete_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    def material(self, name, color):
        return """
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/%s</name>
          </script>
          <ambient>%.3f %.3f %.3f %.3f</ambient>
          <diffuse>%.3f %.3f %.3f %.3f</diffuse>
        </material>
        """ % (name, color.r, color.g, color.b, color.a, color.r, color.g, color.b, color.a)

    def sphere_sdf(self, radius, color):
        return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="marker">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><sphere><radius>%.3f</radius></sphere></geometry>
        %s
      </visual>
    </link>
  </model>
</sdf>""" % (radius, self.material("White", color))

    def uav_sdf(self):
        return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="actual_uav">
    <static>false</static>
    <link name="body">
      <inertial><mass>1.0</mass><inertia><ixx>0.03</ixx><iyy>0.03</iyy><izz>0.05</izz></inertia></inertial>
      <visual name="body_visual">
        <geometry><box><size>0.42 0.42 0.10</size></box></geometry>
        <material><ambient>0.05 0.32 0.90 1</ambient><diffuse>0.05 0.32 0.90 1</diffuse></material>
      </visual>
      <visual name="arm_x">
        <pose>0 0 0.02 0 0 0.785398</pose>
        <geometry><box><size>0.95 0.05 0.04</size></box></geometry>
        <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material>
      </visual>
      <visual name="arm_y">
        <pose>0 0 0.025 0 0 -0.785398</pose>
        <geometry><box><size>0.95 0.05 0.04</size></box></geometry>
        <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""

    def pose(self, x, y, z, yaw=0.0):
        p = Pose()
        p.position.x, p.position.y, p.position.z = x, y, z
        q = quaternion_from_euler(0.0, 0.0, yaw)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        return p

    def safe_delete(self, name):
        try:
            self.delete_srv(name)
        except Exception:
            pass

    def spawn(self, name, sdf, pose):
        self.safe_delete(name)
        self.spawn_srv(name, sdf, "", pose, "world")

    def set_model(self, name, pose):
        state = ModelState()
        state.model_name = name
        state.pose = pose
        state.reference_frame = "world"
        try:
            self.set_state_srv(state)
        except Exception as exc:
            rospy.logwarn("set_model_state failed for %s: %s", name, exc)

    def nominal(self, t):
        if t < self.warmup_time:
            return 0.0, 0.0, self.takeoff_height
        theta = self.angular_rate * (t - self.warmup_time)
        return self.radius * (math.cos(theta) - 1.0), self.radius * math.sin(theta), self.takeoff_height

    def attack(self, t):
        if t < self.attack_start or t > self.attack_end:
            return 0.0, 0.0, 0.0
        tau = t - self.attack_start
        return 1.2 + 0.015 * tau, 0.8, 0.0

    def setup_scene(self):
        self.spawn("actual_uav", self.uav_sdf(), self.pose(0, 0, 0.1))
        self.spawn("nominal_marker", self.sphere_sdf(0.12, rgba(0.45, 0.45, 0.45, 1)), self.pose(0, 0, self.takeoff_height))
        self.spawn("attacked_marker", self.sphere_sdf(0.12, rgba(1.0, 0.45, 0.0, 1)), self.pose(0, 0, self.takeoff_height))

        # Static sampled reference paths: gray nominal, orange attacked during attack window.
        for idx, t in enumerate([i * 1.5 for i in range(int(self.total_time / 1.5) + 1)]):
            nx, ny, nz = self.nominal(t)
            self.spawn("nominal_path_%03d" % idx, self.sphere_sdf(0.045, rgba(0.55, 0.55, 0.55, 0.75)), self.pose(nx, ny, nz))
            ax, ay, az = self.attack(t)
            if abs(ax) + abs(ay) + abs(az) > 1e-6:
                self.spawn("attacked_path_%03d" % idx, self.sphere_sdf(0.055, rgba(1.0, 0.45, 0.0, 0.85)),
                           self.pose(nx + ax, ny + ay, nz + az))

        rospy.loginfo("Gazebo preview legend: gray=nominal path, orange=attacked setpoint, blue=actual UAV/trail.")

    def run(self):
        self.setup_scene()
        rate = rospy.Rate(self.rate_hz)
        start = time.monotonic()
        trail_idx = 0
        last_trail = 0.0

        while not rospy.is_shutdown():
            t = time.monotonic() - start
            if t > self.total_time:
                rospy.loginfo("Gazebo attack preview finished.")
                break

            nx, ny, nz = self.nominal(t)
            ax, ay, az = self.attack(t)
            tx, ty, tz = nx + ax, ny + ay, nz + az

            dt = 1.0 / self.rate_hz
            tau_response = 1.2
            alpha = min(1.0, dt / tau_response)
            self.actual[0] += alpha * (tx - self.actual[0])
            self.actual[1] += alpha * (ty - self.actual[1])
            self.actual[2] += alpha * (tz - self.actual[2])

            self.set_model("nominal_marker", self.pose(nx, ny, nz))
            self.set_model("attacked_marker", self.pose(tx, ty, tz))
            self.set_model("actual_uav", self.pose(self.actual[0], self.actual[1], self.actual[2], 0.0))

            if t - last_trail > 0.55:
                self.spawn("actual_trail_%03d" % trail_idx, self.sphere_sdf(0.06, rgba(0.05, 0.32, 0.90, 0.9)),
                           self.pose(self.actual[0], self.actual[1], self.actual[2]))
                trail_idx += 1
                last_trail = t

            if abs(t - self.attack_start) < dt:
                rospy.loginfo("FDI ATTACK ON: command setpoint is biased.")
            if abs(t - self.attack_end) < dt:
                rospy.loginfo("FDI ATTACK OFF: command setpoint returns to nominal.")

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("gazebo_attack_preview")
    GazeboAttackPreview().run()
