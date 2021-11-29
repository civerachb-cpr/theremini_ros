#!/usr/bin/env python
"""
Plays a song defined in a yaml file
"""

import rospy
from threading import Thread
import random

from std_msgs.msg import Bool
from theremini_player.antenna_control import AntennaControl

def start_stop_callback(data, arg):
    arg.start_stop_callback(data)

class Note:
    def __init__(self, note, duration):
        self.note = note
        self.duration = float(duration)

class SongPlayer:
    def __init__(self):
        self.is_playing = False
        self.loadSong()

        self.volume_antenna = AntennaControl(mode='volume_control')
        self.pitch_antenna = AntennaControl(mode='pitch_control')

    def loadSong(self):
        self.song_name = rospy.get_param('/song/name', None)
        self.theremin_key = rospy.get_param('/song/key', 'c')
        self.theremin_scale = rospy.get_param('/song/scale', 'chromatic')
        self.notes = rospy.get_param('/song/notes', [])
        self.bpm = int(rospy.get_param('/song/bpm', 60))

        self.root_distance = float(rospy.get_param('/song/root_distance', 0.25))
        self.note_scale = float(rospy.get_param('/song/note_scale', 0.01))
        self.root = rospy.get_param('/song/root', 'c4')

        self.volume = rospy.get_param('/song/volume', 0.1)

        # parse the Notes array into actual objects instead of the raw dictionary rospy gives us
        notes = []
        for n in self.notes:
            notes.append(Note(n['note'], n['duration']))
        self.notes = notes
        

    def run(self):
        self.run_sub = rospy.Subscriber('start_stop_theremin', Bool, start_stop_callback, callback_args=self)

        self.volume_antenna.home()
        self.pitch_antenna.home()

        while not rospy.is_shutdown():
            if not self.is_running:
                rospy.sleep(0.1)
            else:
                self.start_song()

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

    def start_song(self):
        next_note = 0

        self.last_note = None
        while self.is_running and next_note < len(self.notes) and not rospy.is_shutdown():
            self.play_note(self.notes[next_note])
            self.last_note = self.notes[next_note]
            next_note += 1

    def play_note(self, note):
        rospy.logdebug("Playing {0} for {1}".format(note.note, note.duration))

        # if we're playing the same note twice in a row, just dip the volume down and back up
        if self.last_note == note:
            self.volume_antenna.move_to(0.0, wait=True)
            self.volume_antenna.move_to(self.volume, wait=True)
        else:
            position = self.note_to_position(note)
            if note['volume'] != None:
                self.volume_antenna.move_to(note['volume'], wait=True)
            else:
                self.pitch_antenna.move_to(note['pitch'], wait=True)

        rospy.sleep(self.bpm/60.0 * note.duration)


    def note_to_position(self, note):
        """
        Convert a note to a distance from the theremin's pitch antenna
        """

        if note == '-':
            return {
                'pitch': None,
                'volume': 0.0
            }

        # Hard-coded G-Maj (Ionian) for Jingle Bells
        notes = {
            'g4': -7,
            'a4': -6,
            'b4': -5,
            'c5': -4,
            'd5': -3,
            'e5': -2,
            'f5': -1,
            'g5': 0,
            'a5': 1,
            'b5': 2,
            'c6': 3,
            'd6': 4,
            'e6': 5,
            'f6': 6,
            'g6': 7
        }

        distance = self.root_distance + notes[note] * self.note_scale

        return {
            'pitch': distance,
            'volume': None
        }
