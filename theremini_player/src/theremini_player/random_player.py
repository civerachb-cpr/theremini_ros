#!/usr/bin/env python

import rospy
from threading import Thread
import random

from std_msgs.msg import Bool
from theremini_player.antenna_control import AntennaControl

def rand_range(min, max):
    return random.random() * (max - min) + min

def start_stop_callback(data, arg):
    arg.start_stop_callback(arg)

class RandomPlayer:
    def __init__(self):
        self.volume_antenna = AntennaControl(mode='volume_control')
        self.pitch_antenna = AntennaControl(mode='pitch_control')

        self.is_playing = False

    def run(self):
        self.pitch_thread = Thread(target=self.pitch_thread_fn)
        self.volume_thread = Thread(target=self.volume_thread_fn)

        self.run_sub = rospy.Subscriber('start_stop', Bool, start_stop_callback, callback_args=self)

        self.pitch_thread.start()
        self.volume_thread.start()
        rospy.spin()

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

    def pitch_thread_fn(self):
        rospy.logdebug("Starting pitch-control thread")

        self.pitch_antenna.home()

        while not rospy.is_shutdown:
            if self.is_playing:
                pitch = rand_range(self.pitch_antenna.min_distance, self.pitch_antenna.max_distance)
                self.pitch_antenna.move_to(pitch, wait=True)
            else:
                rospy.sleep(0.1)

    def volume_thread_fn(self):
        rospy.logdebug("Starting volume-control thread")

        self.volume_antenna.home()

        while not rospy.is_shutdown():
            if self.is_playing:
                volume = rand_range(self.volume_antenna.min_distance, self.volume_antenna.max_distance)
                self.volume_antenna.move_to(volume, wait=True)
            else:
                rospy.sleep(0.1)
