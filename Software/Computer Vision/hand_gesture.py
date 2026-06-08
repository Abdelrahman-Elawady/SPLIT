import cv2
import mediapipe as mp
import requests
import time

ROBOT_IP = "192.168.4.1"
ROBOT_ID = 1
COMMAND_PERIOD = 0.2

last_command = ""
last_sent_time = 0

mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

def send_command(command):
    global last_command, last_sent_time

    now = time.time()

    if command == last_command and now - last_sent_time < COMMAND_PERIOD:
        return

    try:
        url = f"http://{ROBOT_IP}/cmd"
        requests.get(
            url,
            params={"robot": ROBOT_ID, "move": command},
            timeout=0.2,
        )
        print("Sent:", command)
        last_command = command
        last_sent_time = now
    except requests.RequestException:
        print("ESP32 not reachable")

def fingers_up(hand_landmarks):
    tips = [8, 12, 16, 20]   
    pips = [6, 10, 14, 18]   
    result = []

    for tip, pip in zip(tips, pips):
        if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[pip].y:
            result.append(1)
        else:
            result.append(0)

    return result

def classify_gesture(hand_landmarks):
    finger_count = sum(fingers_up(hand_landmarks))

    if finger_count == 0:
        return "IDLE"
    if finger_count == 1:
        return "FORWARD"
    if finger_count == 2:
        return "BACKWARD"
    if finger_count == 3:
        return "RIGHT"
    if finger_count == 4:
        return "LEFT"

    return "IDLE"

#url = "https://192.168.4.4:8080/video"
cap = cv2.VideoCapture(1)

with mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
) as hands:

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        result = hands.process(rgb)

        command = "IDLE"

        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                command = classify_gesture(hand_landmarks)

        send_command(command)

        cv2.putText(frame, command, (30, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)

        cv2.imshow("Gesture Control", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cap.release()
cv2.destroyAllWindows()
