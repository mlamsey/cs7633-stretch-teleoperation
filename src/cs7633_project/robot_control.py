from enum import Enum
from abc import ABC

import mediapipe as mp

from cs7633_project.hand_tracker import HandAnalyzer

LANDMARK = mp.solutions.hands.HandLandmark

############################################################
# Enum Classes
class DriveControlAction(Enum):
    # these states correspond to drive commands
    IDLE = 0
    FORWARD = 1
    BACKWARD = 2
    TURN_CW = 3
    TURN_CCW = 4

class ManipulationControlAction(Enum):
    # these states correspond to cartesian controls
    IDLE = 0
    UP = 1
    DOWN = 2
    LEFT = 3
    RIGHT = 4
    FORWARD = 5
    BACKWARD = 6
    GRASP = 7
    RELEASE = 8

############################################################
# Abstract Base Class
class RobotControl(ABC):
    def __init__(self, debug) -> None:
        super().__init__()
        self.debug = debug

    def set_debug(self, bool_debug):
        self.debug = bool_debug

    def debug_print(self, msg):
        if self.debug:
            print(msg)

    def get_manipulation_action(self, user_input):
        return ManipulationControlAction.IDLE
    
    def get_drive_action(self, user_input):
        return DriveControlAction.IDLE

############################################################
# Control Classes
class HandControl(RobotControl):
    def __init__(self, debug=True) -> None:
        super().__init__(debug)
        self.hand_analyzer = HandAnalyzer()

    def get_manipulation_action(self, user_input):
        action = ManipulationControlAction.IDLE

        extended_fingers = self.hand_analyzer.get_extended_fingers(user_input)
        if extended_fingers is not None:
            if len(extended_fingers) == 1:
                finger = extended_fingers[0]
                if finger == LANDMARK.INDEX_FINGER_TIP:
                    action = ManipulationControlAction.FORWARD
                    # action = ManipulationControlAction.UP
                elif finger == LANDMARK.MIDDLE_FINGER_TIP:
                    action = ManipulationControlAction.LEFT
                elif finger == LANDMARK.RING_FINGER_TIP:
                    action = ManipulationControlAction.BACKWARD
                elif finger == LANDMARK.PINKY_TIP:
                    action = ManipulationControlAction.RIGHT
                elif finger == LANDMARK.THUMB_TIP:
                    # action = ManipulationControlAction.GRASP
                    action = ManipulationControlAction.DOWN
            elif len(extended_fingers) == 2:
                action = ManipulationControlAction.UP
        
        self.debug_print(action.name)
        return action
    
    def get_drive_action(self, user_input):
        return super().get_drive_action(user_input)

class XboxControl(RobotControl):
    def __init__(self, debug) -> None:
        super().__init__(debug)

    def get_manipulation_action(self, user_input):
        return super().get_manipulation_action(user_input)
    
    def get_drive_action(self, user_input):
        return super().get_drive_action(user_input)
    
class UIControl(RobotControl):
    def __init__(self, debug) -> None:
        super().__init__(debug)

    def get_manipulation_action(self, user_input):
        return super().get_manipulation_action(user_input)
    
    def get_drive_action(self, user_input):
        return super().get_drive_action(user_input)
