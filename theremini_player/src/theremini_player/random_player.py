#!/usr/bin/env python

import rospy
from threading import Thread
import random

from std_msgs.msg import Bool
from theremini_player.antenna_control import AntennaControl

def rand_range(min, max):
    return random.random() * (max - min) + min

def start_stop_callback(data, arg):
    arg.start_stop_callback(data)

class RandomPlayer:
    def __init__(self):
        self.volume_antenna = AntennaControl(mode='volume_control')
        self.pitch_antenna = AntennaControl(mode='pitch_control')

        self.is_playing = False

    def run(self):
        self.run_sub = rospy.Subscriber('start_stop_theremin', Bool, start_stop_callback, callback_args=self)

        self.volume_antenna.home()
        self.pitch_antenna.home()

        notes_before_volume_change = int(rand_range(1,8))

        while not rospy.is_shutdown():
            if not self.is_playing:
                rospy.sleep(0.1)
            else:
                pitch = rand_range(self.pitch_antenna.min_distance, self.pitch_antenna.max_distance)
                self.pitch_antenna.move_to(pitch, wait=True)
                notes_before_volume_change = notes_before_volume_change - 1

                if notes_before_volume_change == 0:
                    volume = rand_range(self.volume_antenna.min_distance, self.volume_antenna.max_distance)
                    self.volume_antenna.move_to(volume, wait=True)
                    notes_before_volume_change = int(rand_range(1,8))


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
