import math
import time

import cv2
import requests


ROBOT_IP = "192.168.4.1"

ARUCO_TYPE = cv2.aruco.DICT_5X5_250
MARKER_TO_ROBOT = {
    1: 1,
    2: 2,
}

SEND_COMMANDS = True
COMMAND_PERIOD = 0.3

EDGE_MARGIN_PX = 90
TOO_CLOSE_PX = 230
DEFAULT_TURN = 50
ANGLE_DEADZONE_DEG = 45
WAYPOINT_RADIUS_PX = 130
ROBOT_PATHS = {
    1: [(250, 180), (1050, 180), (1050, 540), (250, 540)],
    #2: [(1050, 540), (250, 540), (250, 180), (1050, 180)],
}


last_sent = {}
path_index = {robot_id: 0 for robot_id in MARKER_TO_ROBOT.values()}


def send_move(robot_id, move):
    now = time.time()
    if last_sent.get(robot_id, ("", 0))[0] == move and now - last_sent[robot_id][1] < COMMAND_PERIOD:
        return

    last_sent[robot_id] = (move, now)
    print(f"Robot {robot_id}: {move}")

    if not SEND_COMMANDS:
        return

    try:
        requests.get(
            f"http://{ROBOT_IP}/cmd",
            params={"robot": robot_id, "move": move, "turn": DEFAULT_TURN},
            timeout=0.2,
        )
    except requests.RequestException:
        print("ESP32 bridge not reachable")


def marker_center(marker_corner):
    corners = marker_corner.reshape((4, 2))
    x = int(corners[:, 0].mean())
    y = int(corners[:, 1].mean())
    return x, y


def marker_heading(marker_corner):
    corners = marker_corner.reshape((4, 2))
    top_left, top_right, bottom_right, bottom_left = corners
    center = corners.mean(axis=0)
    front_midpoint = (top_left + top_right) / 2
    heading = front_midpoint - center
    length = math.hypot(float(heading[0]), float(heading[1]))

    if length == 0:
        return 0.0, -1.0

    return float(heading[0] / length), float(heading[1] / length)


def detect_markers(frame, detector):
    corners, ids, _ = detector.detectMarkers(frame)
    markers = {}

    if ids is None:
        return markers, corners, ids

    for marker_corner, marker_id in zip(corners, ids.flatten()):
        if marker_id not in MARKER_TO_ROBOT:
            continue

        markers[marker_id] = {
            "robot": MARKER_TO_ROBOT[marker_id],
            "center": marker_center(marker_corner),
            "heading": marker_heading(marker_corner),
            "corner": marker_corner,
        }

    return markers, corners, ids


def edge_vector(center, width, height):
    x, y = center

    if x < EDGE_MARGIN_PX:
        return (width / 2) - x, (height / 2) - y
    if x > width - EDGE_MARGIN_PX:
        return (width / 2) - x, (height / 2) - y
    if y < EDGE_MARGIN_PX:
        return (width / 2) - x, (height / 2) - y
    if y > height - EDGE_MARGIN_PX:
        return (width / 2) - x, (height / 2) - y

    return None


def command_from_heading(heading, desired_vector):
    desired_x, desired_y = desired_vector
    desired_len = math.hypot(desired_x, desired_y)

    if desired_len == 0:
        return "IDLE"

    desired_x /= desired_len
    desired_y /= desired_len

    dot = heading[0] * desired_x + heading[1] * desired_y
    cross = heading[0] * desired_y - heading[1] * desired_x
    angle_error = math.degrees(math.atan2(cross, dot))

    if abs(angle_error) <= ANGLE_DEADZONE_DEG:
        return "FORWARD"
    if abs(angle_error) >= 180 - ANGLE_DEADZONE_DEG:
        return "BACKWARD"
    if angle_error > 0:
        return "RIGHT"

    return "LEFT"


def collision_vectors(markers):
    if 1 not in markers or 2 not in markers:
        return {}

    c1 = markers[1]["center"]
    c2 = markers[2]["center"]
    distance = math.dist(c1, c2)

    if distance >= TOO_CLOSE_PX:
        return {}

    return {
        markers[1]["robot"]: (c1[0] - c2[0], c1[1] - c2[1]),
        markers[2]["robot"]: (c2[0] - c1[0], c2[1] - c1[1]),
    }


def current_waypoint(robot_id):
    path = ROBOT_PATHS.get(robot_id, [])
    if not path:
        return None

    return path[path_index[robot_id] % len(path)]


def path_vector(marker):
    robot_id = marker["robot"]
    waypoint = current_waypoint(robot_id)
    if waypoint is None:
        return None

    center = marker["center"]
    distance = math.dist(center, waypoint)

    if distance < WAYPOINT_RADIUS_PX:
        path_index[robot_id] = (path_index[robot_id] + 1) % len(ROBOT_PATHS[robot_id])
        waypoint = current_waypoint(robot_id)

    return waypoint[0] - center[0], waypoint[1] - center[1]


def decide_commands(markers, width, height):
    commands = {}

    for marker in markers.values():
        desired_vector = edge_vector(marker["center"], width, height)
        if desired_vector:
            commands[marker["robot"]] = command_from_heading(marker["heading"], desired_vector)

    for marker in markers.values():
        robot_id = marker["robot"]
        collision_vector = collision_vectors(markers).get(robot_id)
        if collision_vector:
            commands[robot_id] = command_from_heading(marker["heading"], collision_vector)

    for marker in markers.values():
        robot_id = marker["robot"]
        desired_vector = path_vector(marker)
        if robot_id not in commands and desired_vector:
            commands[robot_id] = command_from_heading(marker["heading"], desired_vector)

    return commands


def draw_status(frame, markers, commands):
    height, width = frame.shape[:2]

    cv2.rectangle(
        frame,
        (EDGE_MARGIN_PX, EDGE_MARGIN_PX),
        (width - EDGE_MARGIN_PX, height - EDGE_MARGIN_PX),
        (255, 200, 0),
        2,
    )

    if markers:
        cv2.aruco.drawDetectedMarkers(frame, [m["corner"] for m in markers.values()])

    for robot_id, path in ROBOT_PATHS.items():
        if not path:
            continue

        color = (0, 180, 255) if robot_id == 1 else (255, 120, 0)
        for index, point in enumerate(path):
            next_point = path[(index + 1) % len(path)]
            cv2.line(frame, point, next_point, color, 2)
            cv2.circle(frame, point, 6, color, -1)

        target = current_waypoint(robot_id)
        if target:
            cv2.circle(frame, target, 14, color, 3)
            cv2.putText(
                frame,
                f"R{robot_id} target",
                (target[0] + 10, target[1] + 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
            )

    for marker_id, marker in markers.items():
        x, y = marker["center"]
        robot_id = marker["robot"]
        command = commands.get(robot_id, "IDLE")
        heading = marker["heading"]
        heading_end = (int(x + heading[0] * 45), int(y + heading[1] * 45))

        cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
        cv2.arrowedLine(frame, (x, y), heading_end, (255, 0, 255), 3)
        cv2.putText(
            frame,
            f"M{marker_id} R{robot_id}: {command}",
            (x + 8, y - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

    cv2.putText(
        frame,
        "SEND_COMMANDS=" + str(SEND_COMMANDS),
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2,
    )


def main():
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_TYPE)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    cap = cv2.VideoCapture(0) # cv2.CAP_DSHOW
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        height, width = frame.shape[:2]
        markers, _, _ = detect_markers(frame, detector)
        commands = decide_commands(markers, width, height)

        for robot_id, command in commands.items():
            send_move(robot_id, command)

        draw_status(frame, markers, commands)
        cv2.imshow("Robot ArUco Guard", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
