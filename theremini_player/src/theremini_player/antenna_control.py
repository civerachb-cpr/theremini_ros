#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from tf.transformations import quaterion_from_euler

from math import pi

import geometry_msgs.msg
from std_msgs.msg import String

import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list

class AntennaControl:
    def __init__(self, mode="pitch_control"):
        """
        mode must be one of 'pitch_control' or 'volume_control'
        """
        self.mode = mode
        self.load_params()

        self.commander = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(self.move_group_name)

        self.reference_frame = self.move_group.get_planning_frame()
        self.ee_link = group.get_end_effector_link()

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def load_params(self):
        self.move_group_name = rospy.get_param('theremin/{0}/move_group'.format(self.mode))
        self.antenna_frame = rospy.get_param('theremin/{0}/antenna_frame'.format(self.mode))
        self.min_distance = float(rospy.get_param('theremin/{0}/min_distance'.format(self.mode)))
        self.max_distance = float(rospy.get_param('theremin/{0}/max_distance'.format(self.mode)))
        self.control_axis = rospy.get_param('theremin/{0}/control_axis'.format(self.mode))
        self.control_mode = rospy.get_param('theremin/{0}/control_mode'.format(self.mode))

        self.x_offset = float(rospy.get_param('theremin/{0}/x_offset'.format(self.mode)))
        self.y_offset = float(rospy.get_param('theremin/{0}/y_offset'.format(self.mode)))
        self.z_offset = float(rospy.get_param('theremin/{0}/z_offset'.format(self.mode)))
        self.roll_offset = float(rospy.get_param('theremin/{0}/roll_offset'.format(self.mode)))
        self.pitch_offset = float(rospy.get_param('theremin/{0}/pitch_offset'.format(self.mode)))
        self.yaw_offset = float(rospy.get_param('theremin/{0}/yaw_offset'.format(self.mode)))

        rospy.logdebug(self)

    def __str__(self):
        return "{mode} :: {move_group} {antenna_frame}({control_axis}) [{min_distance}, {max_distance}] {control_mode}".format(
            mode = self.mode,
            move_group = self.move_group_name,
            antenna_frame = self.antenna_frame,
            min_distance = self.min_distance,
            max_distance = self.max_distance,
            control_axis = self.control_axis,
            control_mode = self.control_mode
        )

    def get_antenna_pose(self):
        """
        Get the pose of the antenna relative to this arm's planning frame
        """
        tf = self.tf_buffer.lookup_transform(self.antenna_frame, self.reference_frame, rospy.Duration(0))
        pose = geometry_msgs.msg.Pose()

        pose.position.x = tf.transform.translation.x
        pose.position.y = tf.transform.translation.y
        pose.position.z = tf.transform.translation.z

        pose.orientation.x = tf.transform.rotation.x
        pose.orientation.y = tf.transform.rotation.y
        pose.orientation.z = tf.transform.rotation.z
        pose.orientation.w = tf.transform.rotation.w

        return pose

    def move_to(self, distance, wait=True):
        """
        Move the ee_link to a set distance from the antenna frame
        If wait is True, then we block further execution until the trajectory has stopped
        """

        # create a pose object we'll use to set the ee_link position
        pose = geometry_msgs.msg.Pose()

        # get the position of the antenna we're manipulating
        # and then apply our static RPY offsets
        antenna_pose = self.get_antenna_pose()
        q = geometry_msgs.msg.Quaternion(antenna_pose.orientation.x, antenna_pose.orientation.y,
            antenna_pose.orientation.z, antenna_pose.orientation.w)
        euler_angles = euler_from_quaternion(q)
        euler_angles.x += self.roll_offset
        euler_angles.y += self.pitch_offset
        euler_angles.z += self.yaw_offset
        pose.orientation = quaterion_from_euler(euler_angles)

        # apply the static XYZ offsets relative to the antenna origin
        pose.position.x = antenna_pose.position.x + self.x_offset
        pose.position.y = antenna_pose.position.x + self.y_offset
        pose.position.z = antenna_pose.position.x + self.z_offset

        # apply the desired distance to the correct axis
        if self.control_axis == 'x':
            pose.position.x += distance
        elif self.control_axis == 'y':
            pose.position.y += distance
        elif self.control_axis == 'z':
            pose.position.z += distance
        else:
            rospy.logerr("Unknown control axis {0}".format(self.control_axis))

        # move to the pose calculated above
        rospy.logdebug("{0} target pose\n{1}".format(self.mode, pose))
        self.move_group.set_pose_target(pose)
        plan = self.move_group.go(wait=wait)

        if wait:
            # if we waited, make sure there's no residual movement
            # and clear the planned poses
            self.stop()


    def stop(self):
        self.move_group.stop()
        group.clear_pose_targets()

    def home(self):
        self.move_to(0, wait=True)
