#!/usr/bin/env python3
import rospy

from cs7633_project.xbox_control import XboxControl
from cs7633_project.robot_control import ControllerAction
from cs7633_project.srv import ControlAction, ControlActionRequest

class XboxTeleop:
    def __init__(self) -> None:
        rospy.init_node("xbox_teleop", anonymous=True)
        self.rate = rospy.Rate(10.)
        self.controller = XboxControl()
        self.control_mode = ControlActionRequest.CONTROLLER_MANIPULATION

        # srv
        self.change_robot_pose_proxy = rospy.ServiceProxy(
            "/hri/control_action", ControlAction)

    def swap_control_mode(self):
        rospy.loginfo("Changing Control Mode!")
        if self.control_mode == ControlActionRequest.CONTROLLER_MANIPULATION:
            self.control_mode = ControlActionRequest.CONTROLLER_DRIVE
            rospy.loginfo("Mode: DRIVE")
        elif self.control_mode == ControlActionRequest.CONTROLLER_DRIVE:
            self.control_mode = ControlActionRequest.CONTROLLER_MANIPULATION
            rospy.loginfo("Mode: MANIPULATION")

    def main(self):
        while not rospy.is_shutdown():
            action = self.controller.get_action(self.control_mode)
            if action == ControllerAction.CHANGE_MODE:
                self.swap_control_mode()
                rospy.loginfo("Changing Control Mode!")
                self.rate.sleep()
                continue

            if action is not None:
                # rospy.loginfo(action)

                action = int(action.value)
                state = int(self.control_mode)
                self.change_robot_pose_proxy(action, state)

            self.rate.sleep()


if __name__ == '__main__':
    XboxTeleop().main()
