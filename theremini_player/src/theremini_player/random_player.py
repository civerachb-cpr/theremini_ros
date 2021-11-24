#!/usr/bin/env python

import rospy
from threading import Thread

import random

from theremini_player.antenna_control import AntennaControl

def rand_range(min, max):
    return random.random() * (max - min) + min

class RandomPlayer:
    def __init__(self):
        self.volume_antenna = AntennaControl(mode='volume_control')
        self.pitch_antenna = AntennaControl(mode='pitch_control')

    def run(self):
        self.pitch_thread = Thread(target=self.pitch_thread_fn)
        self.volume_thread = Thread(target=self.volume_thread_fn)

        self.pitch_thread.start()
        self.volume_thread.start()
        rospy.spin()

    def pitch_thread_fn(self):
        rospy.logdebug("Starting pitch-control thread")
        while not rospy.is_shutdown():
            pitch = rand_range(self.pitch_antenna.min_distance, self.pitch_antenna.max_distance)
            self.pitch_antenna.move_to(pitch, wait=True)

    def volume_thread_fn(self):
        rospy.logdebug("Starting volume-control thread")
        while not rospy.is_shutdown():
            volume = rand_range(self.volume_antenna.min_distance, self.volume_antenna.max_distance)
            self.volume_antenna.move_to(volume, wait=True)
