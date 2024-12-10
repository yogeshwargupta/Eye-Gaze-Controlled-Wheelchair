import cv2
import dlib
import numpy as np
from math import hypot
import serial
import time

# Initialize dlib's face detector and facial landmark predictor
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

# Initialize serial communication with Arduino
# Replace 'COMX' with the correct port for your system
arduino = serial.Serial('COM9', 9600, timeout=1)
time.sleep(2)  # Give some time for the connection to establish

# Font for text
font = cv2.FONT_HERSHEY_PLAIN

# Function to calculate midpoint
def midpoint(p1, p2):
    return int((p1.x + p2.x) / 2), int((p1.y + p2.y) / 2)

# Function to calculate blinking ratio
def get_blinking_ratio(eye_points, facial_landmarks):
    left_point = (facial_landmarks.part(eye_points[0]).x, facial_landmarks.part(eye_points[0]).y)
    right_point = (facial_landmarks.part(eye_points[3]).x, facial_landmarks.part(eye_points[3]).y)
    center_top = midpoint(facial_landmarks.part(eye_points[1]), facial_landmarks.part(eye_points[2]))
    center_bottom = midpoint(facial_landmarks.part(eye_points[5]), facial_landmarks.part(eye_points[4]))

    hor_line_length = hypot((left_point[0] - right_point[0]), (left_point[1] - right_point[1]))
    ver_line_length = hypot((center_top[0] - center_bottom[0]), (center_top[1] - center_bottom[1]))

    ratio = hor_line_length / ver_line_length
    return ratio

# Function to calculate gaze ratio
def get_gaze_ratio(eye_points, facial_landmarks, frame, gray):
    left_eye_region = np.array(
        [(facial_landmarks.part(eye_points[0]).x, facial_landmarks.part(eye_points[0]).y),
         (facial_landmarks.part(eye_points[1]).x, facial_landmarks.part(eye_points[1]).y),
         (facial_landmarks.part(eye_points[2]).x, facial_landmarks.part(eye_points[2]).y),
         (facial_landmarks.part(eye_points[3]).x, facial_landmarks.part(eye_points[3]).y),
         (facial_landmarks.part(eye_points[4]).x, facial_landmarks.part(eye_points[4]).y),
         (facial_landmarks.part(eye_points[5]).x, facial_landmarks.part(eye_points[5]).y)],
        np.int32)
    mask = np.zeros(frame.shape[:2], np.uint8)
    cv2.polylines(mask, [left_eye_region], True, 255, 2)
    cv2.fillPoly(mask, [left_eye_region], 255)
    eye = cv2.bitwise_and(gray, gray, mask=mask)

    min_x = np.min(left_eye_region[:, 0])
    max_x = np.max(left_eye_region[:, 0])
    min_y = np.min(left_eye_region[:, 1])
    max_y = np.max(left_eye_region[:, 1])
    gray_eye = eye[min_y: max_y, min_x: max_x]

    _, threshold_eye = cv2.threshold(gray_eye, 70, 255, cv2.THRESH_BINARY)
    height, width = threshold_eye.shape

    left_side_threshold = threshold_eye[:, :width // 2]
    right_side_threshold = threshold_eye[:, width // 2:]

    left_side_white = cv2.countNonZero(left_side_threshold)
    right_side_white = cv2.countNonZero(right_side_threshold)

    if left_side_white == 0:
        gaze_ratio = 1
    elif right_side_white == 0:
        gaze_ratio = 5
    else:
        gaze_ratio = left_side_white / right_side_white

    return gaze_ratio

# Initialize webcam
cap = cv2.VideoCapture(0)
previous_direction = None

while True:
    _, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = detector(gray)
    direction = "STOP"
    color = (0, 0, 255)  # Default color: Red for STOP

    for face in faces:
        landmarks = predictor(gray, face)

        # Blinking ratio for each eye
        left_eye_ratio = get_blinking_ratio([36, 37, 38, 39, 40, 41], landmarks)
        right_eye_ratio = get_blinking_ratio([42, 43, 44, 45, 46, 47], landmarks)

        # Determine if eyes are blinking
        left_eye_closed = left_eye_ratio > 5.7
        right_eye_closed = right_eye_ratio > 5.7

        # Gaze ratio
        gaze_ratio_left_eye = get_gaze_ratio([36, 37, 38, 39, 40, 41], landmarks, frame, gray)
        gaze_ratio_right_eye = get_gaze_ratio([42, 43, 44, 45, 46, 47], landmarks, frame, gray)
        gaze_ratio = (gaze_ratio_left_eye + gaze_ratio_right_eye) / 2

        # Determine direction
        if left_eye_closed and right_eye_closed:
            direction = "STOP"
            color = (0, 0, 255)  # Red
        elif left_eye_closed and not right_eye_closed:
            direction = "BACK"
            color = (255, 255, 0)  # Yellow
        elif right_eye_closed and not left_eye_closed:
            direction = "BACK"
            color = (255, 255, 0)  # Yellow
        else:
            if gaze_ratio < 0.85:
                direction = "RIGHT"
                color = (255, 255, 0)  # Cyan
            elif 0.85 <= gaze_ratio <= 2.5:
                direction = "FORWARD"
                color = (0, 255, 0)  # Green
            elif gaze_ratio > 2.5:
                direction = "LEFT"
                color = (255, 0, 0)  # Blue

        # Send direction to Arduino only if it changes
        if direction != previous_direction:
            arduino.write(direction.encode())  # Send direction as bytes
            print(f"Sent to Arduino: {direction}")
            time.sleep(1)  # Pause for 1 second
            previous_direction = direction

        # Draw bounding boxes around the eyes
        left_eye_region = [(landmarks.part(i).x, landmarks.part(i).y) for i in range(36, 42)]
        right_eye_region = [(landmarks.part(i).x, landmarks.part(i).y) for i in range(42, 48)]

        left_x_min = min([point[0] for point in left_eye_region])
        left_x_max = max([point[0] for point in left_eye_region])
        left_y_min = min([point[1] for point in left_eye_region])
        left_y_max = max([point[1] for point in left_eye_region])

        right_x_min = min([point[0] for point in right_eye_region])
        right_x_max = max([point[0] for point in right_eye_region])
        right_y_min = min([point[1] for point in right_eye_region])
        right_y_max = max([point[1] for point in right_eye_region])

        cv2.rectangle(frame, (left_x_min, left_y_min), (left_x_max, left_y_max), color, 2)
        cv2.rectangle(frame, (right_x_min, right_y_min), (right_x_max, right_y_max), color, 2)

        # Display direction
        cv2.putText(frame, f"Direction: {direction}", (50, 100), font, 2, color, 3)

    # Display frame
    cv2.imshow("Eye Movement Control", frame)

    # Break on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
arduino.close()
cv2.destroyAllWindows()
