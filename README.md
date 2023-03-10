# CS7633 Human-Robot Interaction

Course Project for CS7633 Human-Robot Interaction at the Georgia Institute of Technology

### Team Members
Matthew Lamsey, Houriyeh Majditehran, Chuxuan (Sophie) Yang

# Description

We present a framework for controlling a Hello Robot Stretch using hand gestures as an input. We leverage [open-source hand pose estimation software](https://google.github.io/mediapipe/) to collect control input from a camera mounted on a desk near the robot. This allows users to control a robot for pick-and-place tasks in an intuitive manner, without the need for peripheral input hardware.

# Launch

Three terminals:

1. `roslaunch cs7633_project stretch_core.launch`
2. `rosrun cs7633_project robot_control.py`
3. `rosrun cs7633_project xbox_teleop.py`

# Installation

The generic python framework for this project is (hopefully mostly) system agnostic. The ROS interface requires a ROS installation, and the Hello Robot Stretch controller requires the Stretch's dependencies.

## Python Dependencies

The `src` for this project depends on Google's Mediapipe (https://google.github.io/mediapipe/getting_started/python.html).

Consider using a [virtual environment](https://docs.conda.io/en/latest/) during your development. Install mediapipe by running:

`pip3 install mediapipe`

## ROS

The ROS nodes for this project are written using [ROS 1 Noetic](http://wiki.ros.org/noetic), and have been tested on Ubuntu 20.04.

## Hello Robot Stretch

The Stretch controller is intended to run on a [Hello Robot Stretch RE1](https://hello-robot.com/). The code assumes that your system has the dependencies that are configured in the Hello Robot [robot installation process](https://docs.hello-robot.com/0.2/stretch-install/docs/robot_install/).

# Testing

## Mediapipe

After installing `mediapipe`, you can run the example script from [here](https://google.github.io/mediapipe/solutions/hands#python-solution-api) (copied below):

```
import cv2
import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

# For static images:
IMAGE_FILES = []
with mp_hands.Hands(
    static_image_mode=True,
    max_num_hands=2,
    min_detection_confidence=0.5) as hands:
  for idx, file in enumerate(IMAGE_FILES):
    # Read an image, flip it around y-axis for correct handedness output (see
    # above).
    image = cv2.flip(cv2.imread(file), 1)
    # Convert the BGR image to RGB before processing.
    results = hands.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

    # Print handedness and draw hand landmarks on the image.
    print('Handedness:', results.multi_handedness)
    if not results.multi_hand_landmarks:
      continue
    image_height, image_width, _ = image.shape
    annotated_image = image.copy()
    for hand_landmarks in results.multi_hand_landmarks:
      print('hand_landmarks:', hand_landmarks)
      print(
          f'Index finger tip coordinates: (',
          f'{hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x * image_width}, '
          f'{hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y * image_height})'
      )
      mp_drawing.draw_landmarks(
          annotated_image,
          hand_landmarks,
          mp_hands.HAND_CONNECTIONS,
          mp_drawing_styles.get_default_hand_landmarks_style(),
          mp_drawing_styles.get_default_hand_connections_style())
    cv2.imwrite(
        '/tmp/annotated_image' + str(idx) + '.png', cv2.flip(annotated_image, 1))
    # Draw hand world landmarks.
    if not results.multi_hand_world_landmarks:
      continue
    for hand_world_landmarks in results.multi_hand_world_landmarks:
      mp_drawing.plot_landmarks(
        hand_world_landmarks, mp_hands.HAND_CONNECTIONS, azimuth=5)

# For webcam input:
cap = cv2.VideoCapture(0)
with mp_hands.Hands(
    model_complexity=0,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as hands:
  while cap.isOpened():
    success, image = cap.read()
    if not success:
      print("Ignoring empty camera frame.")
      # If loading a video, use 'break' instead of 'continue'.
      continue

    # To improve performance, optionally mark the image as not writeable to
    # pass by reference.
    image.flags.writeable = False
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = hands.process(image)

    # Draw the hand annotations on the image.
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if results.multi_hand_landmarks:
      for hand_landmarks in results.multi_hand_landmarks:
        mp_drawing.draw_landmarks(
            image,
            hand_landmarks,
            mp_hands.HAND_CONNECTIONS,
            mp_drawing_styles.get_default_hand_landmarks_style(),
            mp_drawing_styles.get_default_hand_connections_style())
    # Flip the image horizontally for a selfie-view display.
    cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))
    if cv2.waitKey(5) & 0xFF == 27:
      break
cap.release()

```
