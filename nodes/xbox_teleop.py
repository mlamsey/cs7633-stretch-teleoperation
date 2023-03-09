#!/usr/bin/env python3
import rospy

from cs7633_project.xbox_control import XboxControl

class XboxTeleop:
    def __init__(self) -> None:
        rospy.init_node("xbox_teleop", anonymous=True)
        self.rate = rospy.Rate(10.)
        self.controller = XboxControl()

    def main(self):
        while not rospy.is_shutdown():
            action = self.controller.get_action()
            rospy.loginfo(action)
            self.rate.sleep()


if __name__ == '__main__':
    XboxTeleop().main()
