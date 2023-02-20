from enum import Enum
import mediapipe as mp

from cs7633_project.hand_tracker import HandAnalyzer

LANDMARK = mp.solutions.hands.HandLandmark

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

class RobotControl:
    def __init__(self) -> None:
        self.hand_analyzer = HandAnalyzer()

    def get_action(self, result):
        action = ManipulationControlAction.IDLE

        extended_fingers = self.hand_analyzer.get_extended_fingers(result)
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
        
        print(action.name)
        return action
