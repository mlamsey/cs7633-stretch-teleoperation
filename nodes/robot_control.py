#!/usr/bin/env python3
import rospy
import numpy as np
import threading

# Stretch Imports
STRETCH = True
if STRETCH:
    import hello_helpers.hello_misc as hm
    from sensor_msgs.msg import JointState

# src
from cs7633_project.robot_control import RobotControl, ManipulationControlAction
from cs7633_project.hand_tracker import HandTracker

# more ros
from cs7633_project.srv import ControlAction, ControlActionResponse

# helpers
def truncate(value, joint_range):
    return np.min([np.max([value, joint_range[0]]), joint_range[1]])

# objects
class RobotControlNode:
    def __init__(self) -> None:
        rospy.init_node("robot_control", anonymous=True)
        self.controller = RobotControl()

        # for testing
        self.hand_tracker = HandTracker()
    
    def main(self):
        while not rospy.is_shutdown():
            _, result = self.hand_tracker.get_frame()
            self.controller.get_action(result)

class StretchControlNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.joint_states_lock = threading.Lock()

        # config
        self.LIFT_INCREMENT = 0.025  # m
        self.EXTENSION_INCREMENT = 0.025  # m
        self.YAW_INCREMENT = 0.025  # rad
        self.BASE_INCREMENT = 0.025  # m

        # state
        self.joint_positions = None

        # objects
        # self.controller = RobotControl()
        # self.hand_tracker = HandTracker()

        # subscribers
        self.joint_states_subscriber = rospy.Subscriber(
            "/stretch/joint_states", JointState, self.callback_joint_state
        )

        # srv
        self.change_robot_pose_service = rospy.Service(
            "/hri/control_action",
            ControlAction,
            self.move_service
        )

    # callbacks
    def callback_joint_state(self, data):
        with self.joint_states_lock:
            joint_states = data

        # extract joint positions
        joint_names = ["joint_lift",
                       "wrist_extension",
                       "joint_wrist_yaw"]
        joint_positions = {}
        for joint_name in joint_names:
            joint_positions[joint_name] = joint_states.position[joint_states.name.index(joint_name)]

        # save
        # self.joint_states_raw = joint_states
        self.joint_positions = joint_positions

    # services
    def move_service(self, data):
        action = data.control_action
        pose = self.joint_positions
        if pose is not None:
            if action == ManipulationControlAction.UP.value:
                print('e')
                pose["joint_lift"] += self.LIFT_INCREMENT
            elif action == ManipulationControlAction.DOWN.value:
                pose["joint_lift"] -= self.LIFT_INCREMENT
            elif action == ManipulationControlAction.FORWARD.value:
                pose["wrist_extension"] += self.EXTENSION_INCREMENT
            elif action == ManipulationControlAction.BACKWARD.value:
                pose["wrist_extension"] -= self.EXTENSION_INCREMENT
            elif action == ManipulationControlAction.LEFT.value:
                pose = {"translate_mobile_base": self.BASE_INCREMENT}  # TODO: wait for this to finish
            elif action == ManipulationControlAction.RIGHT.value:
                pose = {"translate_mobile_base": -self.BASE_INCREMENT}  # TODO: wait for this to finish
            
            self.move(pose)
            return ControlActionResponse(result=True)
        else:
            return ControlActionResponse(result=False)

    # helpers
    def move(self, pose, return_before_done=False):
        # move while enforcing joint limits
        # limits
        LIFT_LIMITS = [0.05, 1.05]
        EXTENSION_LIMITS = [0.025, 0.45]
        YAW_LIMITS = 0.75 * np.array([-np.pi, np.pi])
        GRASP_LIMITS = [-0.4, 1.0]  # TODO: check these

        print(self.joint_positions)
        print(pose)
        # truncate
        for key in pose.keys():
            if key == "joint_lift":
                pose[key] = truncate(pose[key], LIFT_LIMITS)
            elif key == "wrist_extension":
                pose[key] = truncate(pose[key], EXTENSION_LIMITS)
            elif key == "joint_wrist_yaw":
                pose[key] = truncate(pose[key], YAW_LIMITS)
            elif key == "joint_gripper_finger_left":
                pose[key] = truncate(pose[key], GRASP_LIMITS)
        
        # go
        self.move_to_pose(pose, return_before_done=return_before_done)

    # main
    def main(self):
        hm.HelloNode.main(self, 'stretch_controller', 'stretch_namespace', wait_for_first_pointcloud=False)
        rospy.spin()

if __name__ == '__main__':
    StretchControlNode().main()