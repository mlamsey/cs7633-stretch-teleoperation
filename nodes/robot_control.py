#!/usr/bin/env python3
import rospy

# Stretch Imports
STRETCH = True
if STRETCH:
    import hello_helpers.hello_misc as hm

# 
from cs7633_project.robot_control import RobotControl, ManipulationControlAction
from cs7633_project.hand_tracker import HandTracker

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

        self.controller = RobotControl()
        self.hand_tracker = HandTracker()

    def main(self):
        hm.HelloNode.main(self, 'stretch_controller', 'stretch_namespace', wait_for_first_pointcloud=False)
        self.rate = rospy.Rate(10.)

        while not rospy.is_shutdown():
            _, result = self.hand_tracker.get_frame()
            action = self.controller.get_action(result)
            if action == ManipulationControlAction.UP:
                self.move_to_pose({"joint_lift": 0.65})
                print('e')
            elif action == ManipulationControlAction.DOWN:
                self.move_to_pose({"joint_lift": 0.45})
            elif action == ManipulationControlAction.FORWARD:
                self.move_to_pose({"wrist_extension": 0.35})
            elif action == ManipulationControlAction.BACKWARD:
                self.move_to_pose({"wrist_extension": 0.05})
            elif action == ManipulationControlAction.LEFT:
                self.move_to_pose({"translate_mobile_base": 0.1})
            elif action == ManipulationControlAction.RIGHT:
                self.move_to_pose({"translate_mobile_base": -0.1})
            self.rate.sleep()

if __name__ == '__main__':
    StretchControlNode().main()