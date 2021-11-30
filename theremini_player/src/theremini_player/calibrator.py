#!/usr/bin/env python

import rospy
from threading import Thread
import random

from std_msgs.msg import Bool
from theremini_player.antenna_control import AntennaControl

def start_stop_callback(data, arg):
    arg.start_stop_callback(data)

def next_step_callback(data, arg):
    arg.next_step_callback(data)

class Calibrator:
    def __init__(self):
        self.volume_antenna = AntennaControl(mode='volume_control')
        self.pitch_antenna = AntennaControl(mode='pitch_control')

        self.STEP_IDLE = 0
        self.STEP_BACK_OFF = 1
        self.STEP_BACK_OFF_DONE = 2
        self.STEP_PITCH_NEAR = 3
        self.STEP_PITCH_NEAR_DONE = 4
        self.STEP_PITCH_FAR = 5
        self.STEP_PITCH_FAR_DONE = 6
        self.STEP_VOLUME_NEAR = 7
        self.STEP_VOLUME_NEAR_DONE = 8
        self.STEP_VOLUME_FAR = 9
        self.STEP_DONE = 10

        self.is_playing = False

    def run(self):
        self.run_sub = rospy.Subscriber('start_calibration', Bool, start_stop_callback, callback_args=self)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.volume_antenna.home()
        self.pitch_antenna.home()

        # idle until we get the start signal
        while not self.is_playing and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.current_step = self.STEP_IDLE
        self.next_step_sub = rospy.Subscriber('next_step', Bool, next_step_callback, callback_args=self)
        rospy.loginfo("[CALIBRATE] Started! Publish `true` ONCE to /next_step to continue")

        while not rospy.is_shutdown() and self.is_playing and self.current_step < self.STEP_DONE:
            if self.current_step == self.STEP_BACK_OFF:
                # drive backwards away from the instrument
                rospy.loginfo("[CALIBRATE] Backing away from the instrument")
                start_time = rospy.Time.now()
                cmd = Twist()
                cmd.linear.x = -0.25
                now = rospy.Time.now()
                while (now - start_time) < rospy.Duration(6):
                    self.cmd_vel_pub.publish(cmd)
                    rospy.sleep(0.1)
                cmd.linear.x = 0.0
                self.cmd_vel_pub.publish(cmd)
                self.current_step = self.BACK_OFF_DONE
                rospy.loginfo("[CALIBRATE] Publish `true` ONCE to /next_step to continue")

            elif self.current_step == self.STEP_PITCH_NEAR:
                rospy.loginfo("[CALIBRATE] Moving pitch hand near the instrument")
                self.pitch_antenna.home()
                self.current_step = self.STEP_PITCH_NEAR_DONE
                rospy.loginfo("[CALIBRATE] Publish `true` ONCE to /next_step to continue")

            elif self.current_step == self.STEP_PITCH_FAR:
                rospy.loginfo("[CALIBRATE] Moving pitch hand far from the instrument")
                self.pitch_antenna.far()
                self.current_step = self.STEP_PITCH_FAR_DONE
                rospy.loginfo("[CALIBRATE] Publish `true` ONCE to /next_step to continue")

            elif self.current_step == self.STEP_VOLUME_NEAR:
                rospy.loginfo("[CALIBRATE] Moving volume hand near the instrument")
                self.pitch_antenna.home()
                self.current_step = self.STEP_VOLUME_NEAR_DONE
                rospy.loginfo("[CALIBRATE] Publish `true` ONCE to /next_step to continue")

            elif self.current_step == self.STEP_VOLUME_FAR:
                rospy.loginfo("[CALIBRATE] Moving volume hand far from the instrument")
                self.pitch_antenna.far()
                self.current_step = self.STEP_DONE
                rospy.loginfo("[CALIBRATE] DONE")

            else:
                rospy.sleep(0.1)


    def next_step_callback(self, data):
        self.current_step += 1


    def start_stop_callback(self, data):
        was_playing = self.is_playing
        self.is_playing = data.data

        if was_playing != self.is_playing:
            if self.is_playing:
                rospy.logdebug("Received start signal!")
            else:
                rospy.logdebug("Received stop signal!")
                pitch_antenna.stop()
                volume_antenna.stop()
