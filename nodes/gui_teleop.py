#!/usr/bin/env python3
import rospy

# system
from PyQt5 import QtWidgets
import sys

# src
from cs7633_project import HRI_GUI
from cs7633_project.robot_control import ManipulationControlAction, DriveControlAction

# ros
from cs7633_project.srv import ControlAction, ControlActionRequest

############################################################
class GUINode:
    def __init__(self) -> None:
        rospy.init_node("gui_control", anonymous=True)

        # self.app = QtWidgets.QApplication([])
        # self.ui_mainwindow = self.setup_gui()

        # srv
        # self.change_robot_pose_proxy = rospy.ServiceProxy(
        #     "/hri/control_action", ControlAction)

    def change_robot_pose_proxy(self, a, s):
        pass

    # GUI Callbacks
    def gui_callback_manipulation_action(self, action_name):
        action = ManipulationControlAction.make(action_name)
        state = ControlActionRequest.CONTROLLER_MANIPULATION
        self.change_robot_pose_proxy(action, state)

    def gui_callback_drive_action(self, action_name):
        action = DriveControlAction.make(action_name)
        state = ControlActionRequest.CONTROLLER_DRIVE
        self.change_robot_pose_proxy(action, state)

    # Main
    def main(self):
        app = QtWidgets.QApplication([])
        MainWindow = QtWidgets.QMainWindow()
        ui = HRI_GUI.Ui_MainWindow()
        ui.setupUi(MainWindow)
        MainWindow.show()
        sys.exit(self.app.exec_())

if __name__ == '__main__':
    GUINode().main()
