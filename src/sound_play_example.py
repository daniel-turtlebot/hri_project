#!/usr/bin/env python

import rospy
from sound_play.libsoundplay import SoundClient

from sound_play.msg import SoundRequest

class SoundPlayExample:

    def __init__(self):
        rospy.init_node('soundplay_example', anonymous=True)

        self.soundhandle = SoundClient()

        # Sleep to make sure everything has time to connect to master
        rospy.sleep(0.5)

    def run(self):
        # Testing with text to speech
        self.soundhandle.say("Hello, I am Turtle Bot")
        # Sleep to let the robot finish speaking
        rospy.sleep(3)
        # Play a built-in sound
        self.soundhandle.play(SoundRequest.NEEDS_PLUGGING)
        rospy.sleep(2)


if __name__ == '__main__':
    spe = SoundPlayExample()
    spe.run()
