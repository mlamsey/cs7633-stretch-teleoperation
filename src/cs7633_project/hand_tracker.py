import cv2
import mediapipe as mp
import numpy as np
from typing import List
from enum import Enum

from cs7633_project.robot_control import ManipulationControlAction, DriveControlAction, ControllerAction
from cs7633_project.srv import ControlActionRequest

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

LANDMARK = mp.solutions.hands.HandLandmark

class Direction(Enum):
    NONE = 0
    UP = 1
    RIGHT = 2
    DOWN = 3
    LEFT = 4

############################################################
# helper functions
def _tip2wrist_dist(tip: LANDMARK, estimate) -> float:
    tip_landmark = estimate[0].landmark[tip.value]
    wrist_landmark = estimate[0].landmark[LANDMARK.WRIST.value]

    t = np.array([tip_landmark.x, tip_landmark.y, tip_landmark.z])
    w = np.array([wrist_landmark.x, wrist_landmark.y, wrist_landmark.z])
    
    dist = np.linalg.norm(t - w)
    return dist

def _tip2wrist_direction(tip: LANDMARK, estimate):
    tip_landmark = estimate[0].landmark[tip.value]
    wrist_landmark = estimate[0].landmark[LANDMARK.WRIST.value]

    t = np.array([tip_landmark.x, tip_landmark.y, tip_landmark.z])
    w = np.array([wrist_landmark.x, wrist_landmark.y, wrist_landmark.z])
    v = t - w

    angle = np.arctan2(v[0], v[1])
    return angle

def _tip2tip_distance(estimate, finger_1, finger_2):
    f1_landmark = estimate[0].landmark[finger_1.value]
    f2_landmark = estimate[0].landmark[finger_2.value]

    f1 = np.array([f1_landmark.x, f1_landmark.y, f1_landmark.z])
    f2 = np.array([f2_landmark.x, f2_landmark.y, f2_landmark.z])
    
    return np.linalg.norm(f2 - f1)

def _get_extended_fingers(estimate, threshold=0.25):
    tips = [LANDMARK.THUMB_TIP,
        LANDMARK.INDEX_FINGER_TIP,
        LANDMARK.MIDDLE_FINGER_TIP,
        LANDMARK.RING_FINGER_TIP,
        LANDMARK.PINKY_TIP]

    distances = [_tip2wrist_dist(tip, estimate) for tip in tips]

    extended = []
    for tip, distance in zip(tips, distances):
        if distance > threshold:
            extended.append(tip)

    return extended

############################################################
class HandAnalyzer:
    # contains methods for analyzing the results of a hand pose estimate
    def __init__(self) -> None:
        pass

    def get_extended_fingers(self, hand_prediction_results, threshold=0.25) -> List[LANDMARK]:
        landmarks = hand_prediction_results.multi_hand_landmarks
        if landmarks is not None:
            return _get_extended_fingers(landmarks, threshold=threshold)
        return []

    def get_finger_direction(self, hand_prediction_results, finger: LANDMARK) -> float:
        landmarks = hand_prediction_results.multi_hand_landmarks
        if landmarks is not None:
            return _tip2wrist_direction(finger, landmarks)
        
        return 0.

    def get_finger_distance(self, hand_prediction_results, finger_1: LANDMARK, finger_2: LANDMARK):
        landmarks = hand_prediction_results.multi_hand_landmarks
        if landmarks is not None:
            return _tip2tip_distance(landmarks, finger_1, finger_2)
        
        return 10.

    def check_fingers_in_contact(self, hand_prediction_results, finger_1: LANDMARK, finger_2: LANDMARK, threshold=0.1):
        if self.get_finger_distance(hand_prediction_results, finger_1, finger_2) < threshold:
            return True
        else:
            return False

class HandTracker(HandAnalyzer):
    # contains interface with a webcam and visualization methods
    def __init__(self) -> None:
        self.capture_device = cv2.VideoCapture(6)  # NOTE: the arg changes. 0 for desktop, 6 for stretch
        self.hands_model = mp_hands.Hands(
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)

    def get_image(self, capture_device):
        success, image = capture_device.read()
        if not success:
            print("Ignoring empty camera frame.")
            return None

        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return image

    def show(self, image, static=False):
        cv2.imshow('MediaPipe Hands', image)
        if static:
            cv2.waitKey(5 * 1000) # wait 5 seconds (5000 ms)
        else:
            cv2.waitKey(5)

    def annotate_hands(self, image, hand_prediction_results):
        # Draw the hand annotations on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if hand_prediction_results.multi_hand_landmarks:
            for hand_landmarks in hand_prediction_results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    image,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style())
        
        # Flip the image horizontally for a selfie-view display.
        cv2.flip(image, 1)
        
        return image

    def get_frame(self):
        image = self.get_image(self.capture_device)
        hand_prediction_results = self.hands_model.process(image)
        image = self.annotate_hands(image, hand_prediction_results)
        return image, hand_prediction_results
    
    def get_manipulation_action(self, hand_prediction_results):
        # get extended fingers
        extended_fingers = self.get_extended_fingers(hand_prediction_results, threshold=0.35)

        # determine action
        n_fingers = len(extended_fingers)

        # grasp
        if self.check_fingers_in_contact(hand_prediction_results, LANDMARK.THUMB_TIP, LANDMARK.INDEX_FINGER_TIP, threshold=0.05):
            return ManipulationControlAction.GRASP

        # single finger actions
        if n_fingers == 1:
            if LANDMARK.INDEX_FINGER_TIP in extended_fingers:
                index_finger_angle = self.get_finger_direction(hand_prediction_results, LANDMARK.INDEX_FINGER_TIP)
                if index_finger_angle < -0.5:
                    return ManipulationControlAction.LEFT
                elif index_finger_angle > 0.5:
                    return ManipulationControlAction.RIGHT
                else:
                    return ManipulationControlAction.FORWARD
            elif LANDMARK.THUMB_TIP in extended_fingers:
                return ManipulationControlAction.BACKWARD
            else:
                return ManipulationControlAction.IDLE
        
        # two finger actions
        elif n_fingers == 2:
            # up / down
            if LANDMARK.INDEX_FINGER_TIP in extended_fingers and LANDMARK.MIDDLE_FINGER_TIP in extended_fingers:
                if self.check_fingers_in_contact(hand_prediction_results, *extended_fingers):
                    return ManipulationControlAction.DOWN
                else:
                    return ManipulationControlAction.UP
            # change modes
            if LANDMARK.INDEX_FINGER_TIP in extended_fingers and LANDMARK.PINKY_TIP in extended_fingers:
                return ControllerAction.CHANGE_MODE
        
        # five finger actions
        elif n_fingers == 5:
            if self.check_fingers_in_contact(hand_prediction_results, LANDMARK.THUMB_TIP, LANDMARK.INDEX_FINGER_TIP, threshold=0.10):
                return ManipulationControlAction.GRASP
            return ManipulationControlAction.RELEASE
        
        return ManipulationControlAction.IDLE
        
    def get_drive_action(self, hand_prediction_results):
        # get extended fingers
        extended_fingers = self.get_extended_fingers(hand_prediction_results, threshold=0.35)

        # determine action
        n_fingers = len(extended_fingers)

        # one finger actions
        if n_fingers == 1:
            if LANDMARK.INDEX_FINGER_TIP in extended_fingers:
                index_finger_angle = self.get_finger_direction(hand_prediction_results, LANDMARK.INDEX_FINGER_TIP)
                if index_finger_angle < -0.5:
                    return DriveControlAction.TURN_CCW
                elif index_finger_angle > 0.5:
                    return DriveControlAction.TURN_CW
                else:
                    return DriveControlAction.FORWARD
            elif LANDMARK.THUMB_TIP in extended_fingers:
                return DriveControlAction.BACKWARD
            else:
                return DriveControlAction.IDLE

        # two finger actions
        elif n_fingers == 2:
            # change modes
            if LANDMARK.INDEX_FINGER_TIP in extended_fingers and LANDMARK.PINKY_TIP in extended_fingers:
                return ControllerAction.CHANGE_MODE
        
        return DriveControlAction.IDLE

    def get_action(self, hand_prediction_results, mode):
        if mode == ControlActionRequest.CONTROLLER_MANIPULATION:
            return self.get_manipulation_action(hand_prediction_results)
        elif mode == ControlActionRequest.CONTROLLER_DRIVE:
            return self.get_drive_action(hand_prediction_results)

        return None

    def run(self):
        while self.capture_device.isOpened():
            image, hand_prediction_results = self.get_frame()
            if image is None:
                continue
            
            self.show(image)
            if cv2.waitKey(5) & 0xFF == 27:
                break

        self.capture_device.release()

if __name__ == '__main__':
    HandTracker().run()