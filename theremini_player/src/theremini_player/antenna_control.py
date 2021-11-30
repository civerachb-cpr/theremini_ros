#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
import tf.transformations

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
        self.ee_link = self.move_group.get_end_effector_link()

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # determine a static quaternion for the hand position
        q = tf.transformations.quaternion_from_euler(self.roll_offset, self.pitch_offset, self.roll_offset)
        self.hand_orientation = geometry_msgs.msg.Quaternion()
        self.hand_orientation.x = q[0]
        self.hand_orientation.y = q[1]
        self.hand_orientation.z = q[2]
        self.hand_orientation.w = q[3]

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
        tf = self.tf_buffer.lookup_transform(self.reference_frame, self.antenna_frame, rospy.Duration(0))
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

        if distance < self.min_distance:
            distance = self.min_distance
        elif distance > self.max_distance:
            distance = self.max_distance

        antenna_pose = self.get_antenna_pose()

        # create a pose object we'll use to set the ee_link position
        pose = geometry_msgs.msg.Pose()

        # apply the static XYZ offsets relative to the antenna origin
        pose.position.x = antenna_pose.position.x + self.x_offset
        pose.position.y = antenna_pose.position.y + self.y_offset
        pose.position.z = antenna_pose.position.z + self.z_offset


        # apply the desired distance to the correct axis
        if self.control_axis == 'x':
            pose.position.x -= distance   # larger distance from the antenna means moving the hand closer!
        elif self.control_axis == 'y':
            pose.position.y += distance
        elif self.control_axis == 'z':
            pose.position.z += distance
        else:
            rospy.logerr("Unknown control axis {0}".format(self.control_axis))

        # orient the hand correctly
        pose.orientation = self.hand_orientation

        # move to the pose calculated above
        rospy.logerr("{0} Goal Pose\n{1}".format(self.mode, pose))
        self.move_group.set_pose_target(pose)
        plan = self.move_group.go(wait=wait)


    def stop(self):
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def home(self):
        self.move_to(self.min_distance, wait=True)

    def far(self):
        self.move_to(self.max_distance, wait=True)
