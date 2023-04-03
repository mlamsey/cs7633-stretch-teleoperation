from __future__ import annotations

from enum import Enum
from abc import ABC
from typing import Optional

import mediapipe as mp

# from cs7633_project.hand_tracker import HandAnalyzer
from cs7633_project.srv import ControlAction, ControlActionRequest

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

    def make(action_name: str) -> Optional[DriveControlAction]:
        name = action_name.upper()
        if not hasattr(DriveControlAction, name):
            return None
        return getattr(DriveControlAction, name)

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

    def make(action_name: str) -> Optional[ManipulationControlAction]:
        name = action_name.upper()
        if not hasattr(ManipulationControlAction, name):
            return None
        return getattr(ManipulationControlAction, name)

class ControllerAction(Enum):
    # these states correspond to actions affecting the controller state
    IDLE = 0
    CHANGE_MODE = 1
    SET_MODE_MANIPULATION = 2
    SET_MODE_DRIVE = 3

############################################################
# Abstract Base Class
class RobotControl(ABC):
    def __init__(self, debug) -> None:
        super().__init__()
        self.debug = debug
        self.controller_state = ControlActionRequest.CONTROLLER_MANIPULATION

    def set_debug(self, bool_debug: bool):
        self.debug = bool_debug

    def set_controller_state(self, state: ControlAction):
        self.controller_state = state

    def debug_print(self, msg):
        if self.debug:
            print(msg)

    def get_manipulation_action(self, user_input) -> ManipulationControlAction:
        return ManipulationControlAction.IDLE
    
    def get_drive_action(self, user_input) -> DriveControlAction:
        return DriveControlAction.IDLE

############################################################
# Control Classes
# class HandControl(RobotControl):
#     def __init__(self, debug=True) -> None:
#         super().__init__(debug)
#         self.hand_analyzer = HandAnalyzer()

#     def get_manipulation_action(self, user_input) -> ManipulationControlAction:
#         action = ManipulationControlAction.IDLE

#         extended_fingers = self.hand_analyzer.get_extended_fingers(user_input)
#         if extended_fingers is not None:
#             if len(extended_fingers) == 1:
#                 finger = extended_fingers[0]
#                 if finger == LANDMARK.INDEX_FINGER_TIP:
#                     action = ManipulationControlAction.FORWARD
#                     # action = ManipulationControlAction.UP
#                 elif finger == LANDMARK.MIDDLE_FINGER_TIP:
#                     action = ManipulationControlAction.LEFT
#                 elif finger == LANDMARK.RING_FINGER_TIP:
#                     action = ManipulationControlAction.BACKWARD
#                 elif finger == LANDMARK.PINKY_TIP:
#                     action = ManipulationControlAction.RIGHT
#                 elif finger == LANDMARK.THUMB_TIP:
#                     # action = ManipulationControlAction.GRASP
#                     action = ManipulationControlAction.DOWN
#             elif len(extended_fingers) == 2:
#                 action = ManipulationControlAction.UP
        
#         self.debug_print(action.name)
#         return action
    
#     def get_drive_action(self, user_input) -> DriveControlAction:
#         return super().get_drive_action(user_input)

class XboxControl(RobotControl):
    def __init__(self, debug) -> None:
        super().__init__(debug)

    def get_manipulation_action(self, user_input) -> ManipulationControlAction:
        return super().get_manipulation_action(user_input)
    
    def get_drive_action(self, user_input) -> DriveControlAction:
        return super().get_drive_action(user_input)
    
class UIControl(RobotControl):
    def __init__(self, debug) -> None:
        super().__init__(debug)

    def get_manipulation_action(self, user_input) -> ManipulationControlAction:
        return super().get_manipulation_action(user_input)
    
    def get_drive_action(self, user_input) -> DriveControlAction:
        return super().get_drive_action(user_input)

class KeyboardControl(RobotControl):
    def __init__(self, debug=True) -> None:
        super().__init__(debug)

        self.manipulation_keyboard_mapping = {
            "u": ManipulationControlAction.UP,
            "d": ManipulationControlAction.DOWN,
            "l": ManipulationControlAction.LEFT,
            "r": ManipulationControlAction.RIGHT,
            "f": ManipulationControlAction.FORWARD,
            "b": ManipulationControlAction.BACKWARD,
        }

        self.drive_keyboard_mapping = {
            "l": DriveControlAction.TURN_CCW,
            "r": DriveControlAction.TURN_CW,
            "f": DriveControlAction.FORWARD,
            "b": DriveControlAction.BACKWARD,
        }

    def print_manipulation_menu(self):
        print(" ===== MANIPULATION MENU ===== ")
        print(" (U) Up          (D) Down ")
        print(" (L) Left        (R) Right")
        print(" (F) Forwards    (B) Backwards ")

    def print_drive_menu(self):
        print(" ===== DRIVE MENU ===== ")
        print(" (L) Turn Left    (R) Turn Right ")
        print(" (F) Forwards     (B) Backwards ")

    def get_selection(self):
        return input("Enter a selection: ").lower()

    def get_manipulation_action(self, user_input) -> ManipulationControlAction:
        valid_entries = self.manipulation_keyboard_mapping.keys()
        if user_input in valid_entries:
            return self.manipulation_keyboard_mapping[user_input]
        else:
            return ManipulationControlAction.IDLE
    
    def get_drive_action(self, user_input) -> DriveControlAction:
        return super().get_drive_action(user_input)