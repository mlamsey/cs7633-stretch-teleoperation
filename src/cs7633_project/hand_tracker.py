import cv2
import mediapipe as mp
import numpy as np
from typing import List

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

LANDMARK = mp.solutions.hands.HandLandmark

############################################################
# helper functions
def _tip2wrist_dist(tip: LANDMARK, estimate):
    tip_landmark = estimate[0].landmark[tip.value]
    wrist_landmark = estimate[0].landmark[LANDMARK.WRIST.value]

    t = np.array([tip_landmark.x, tip_landmark.y, tip_landmark.z])
    w = np.array([wrist_landmark.x, wrist_landmark.y, wrist_landmark.z])
    
    dist = np.linalg.norm(t - w)
    return dist

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
class HandTracker:
    def __init__(self) -> None:
        self.capture_device = cv2.VideoCapture(0)
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

    def get_extended_fingers(self, hand_prediction_results) -> List[LANDMARK]:
        return _get_extended_fingers(hand_prediction_results)

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