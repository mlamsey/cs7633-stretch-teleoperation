#!/usr/bin/env python3
import rospy
import numpy as np

# src
from cs7633_project.robot_control import KeyboardControl, ControllerState

# more ros
from cs7633_project.srv import ControlAction

class KeyboardTeleoperationNode(KeyboardControl):
    def __init__(self) -> None:
        super().__init__()
        rospy.init_node("keyboard_teleop", anonymous=True)
        self.rate = rospy.Rate(10.)

        # srv
        self.change_robot_pose_proxy = rospy.ServiceProxy(
            "/hri/control_action", ControlAction)

    def main(self):
        while not rospy.is_shutdown():
            if self.controller_state == ControllerState.MANIPULATION:
                self.print_manipulation_menu()
            elif self.controller_state == ControllerState.DRIVE:
                self.print_drive_menu()
            else:
                rospy.logwarn("KeyboardTeleoperationNode::main: invalid state, resetting to drive")
                self.state = ControllerState.DRIVE

            ui = self.get_selection()

            if self.controller_state == ControllerState.MANIPULATION:
                action = self.get_manipulation_action(ui)
            elif self.controller_state == ControllerState.DRIVE:
                action = self.get_drive_action(ui)

            # set up service call
            action = int(action.value)
            state = int(self.controller_state.value)
            self.change_robot_pose_proxy(action, state)

            self.rate.sleep()

if __name__ == '__main__':
    KeyboardTeleoperationNode().main()
