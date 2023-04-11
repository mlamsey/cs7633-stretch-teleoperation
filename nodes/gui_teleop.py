#!/usr/bin/env python3
import rospy
import json

# system
from PyQt5 import QtWidgets
import sys

# src
from cs7633_project import HRI_GUI
from cs7633_project.robot_control import ManipulationControlAction, DriveControlAction
from cs7633_project.logger import build_action_log_msg

# ros
from cs7633_project.srv import ControlAction, ControlActionRequest
from std_msgs.msg import String

############################################################
class GUINode:
    def __init__(self) -> None:
        rospy.init_node("gui_control", anonymous=True)

        self.app = QtWidgets.QApplication([])
        self.ui_mainwindow = self.setup_gui()

        # publishers
        self.log_dict_publisher = rospy.Publisher(
            "/hri/log/dict",
            String,
            queue_size=10
        )

        # srv
        self.change_robot_pose_proxy = rospy.ServiceProxy(
            "/hri/control_action", ControlAction)

        self.done_moving = True

    # Helpers
    def setup_gui(self):
        # Build Qt Window
        MainWindow = QtWidgets.QMainWindow()
        ui = HRI_GUI.Ui_MainWindow()
        ui.setupUi(MainWindow)

        # Setup Callbacks: Drive
        ui.backward_drive.clicked.connect(lambda: self.gui_callback_drive_action(DriveControlAction.BACKWARD))
        ui.forward_drive.clicked.connect(lambda: self.gui_callback_drive_action(DriveControlAction.FORWARD))
        ui.left_drive.clicked.connect(lambda: self.gui_callback_drive_action(DriveControlAction.TURN_CCW))
        ui.right_drive.clicked.connect(lambda: self.gui_callback_drive_action(DriveControlAction.TURN_CW))

        # Setup Callbacks: Manipulation
        ui.backward_manipulate.clicked.connect(lambda: self.gui_callback_manipulation_action(ManipulationControlAction.BACKWARD))
        ui.forward_manipulate.clicked.connect(lambda: self.gui_callback_manipulation_action(ManipulationControlAction.FORWARD))
        ui.left_manipulate.clicked.connect(lambda: self.gui_callback_manipulation_action(ManipulationControlAction.LEFT))
        ui.right_manipulate.clicked.connect(lambda: self.gui_callback_manipulation_action(ManipulationControlAction.RIGHT))
        ui.up_manipulate.clicked.connect(lambda: self.gui_callback_manipulation_action(ManipulationControlAction.UP))
        ui.down_manipulate.clicked.connect(lambda: self.gui_callback_manipulation_action(ManipulationControlAction.DOWN))
        ui.grasp.clicked.connect(lambda: self.gui_callback_manipulation_action(ManipulationControlAction.GRASP))
        ui.release.clicked.connect(lambda: self.gui_callback_manipulation_action(ManipulationControlAction.RELEASE))
        
        return MainWindow

    def change_pose(self, action, state):
        try:
            now = rospy.Time.now()
            log_dict = build_action_log_msg("gui", now.secs, now.nsecs, action.value, state)
            self.log_dict_publisher.publish(json.dumps(log_dict))
            self.change_robot_pose_proxy(action.value, state)
        except rospy.ServiceException as e:
            rospy.logerr(e)
            rospy.logerr("Dropping command: " + str(action))

    # GUI Callbacks
    def gui_callback_manipulation_action(self, action: ManipulationControlAction):
        state = ControlActionRequest.CONTROLLER_MANIPULATION
        self.change_pose(action, state)

    def gui_callback_drive_action(self, action: DriveControlAction):
        state = ControlActionRequest.CONTROLLER_DRIVE
        self.change_pose(action, state)

    # Main
    def main(self):
        self.ui_mainwindow.show()
        # MainWindow.show()
        sys.exit(self.app.exec_())

if __name__ == '__main__':
    GUINode().main()
