import math

# Thread info
main_dt = 0.1

gyro_dt = 0.01

mainThreadId = 0
gyroThreadId = 1


# Size info
ARUCO_SIDE_SIZE = 40  # sm
ARUCO_SIDE_SIZE_TO_METRIC = 100  # sm


# Sensors info
SONAR_DIST_NOTHING = 5_000
SONAR_MAX_DIST = 5
SONAR_FAKE_MIN = 0.02

ARUCO_DISTANCE = 0.6
ARUCO_CAMERA_DISTANCE = 2

OBSTACLE_DISTANCE = 1.4
OBSTACLE_SUBTRACT_DISTANCE = 0.4
OBSTACLE_SIDE_DISTANCE = 0.4

imageW = 640
imageH = 480
CAMERA_ANGLE = 45 * math.pi / 180


# Movement info
MOVEMENT_SPEED = 3
LOW_MOVEMENT_SPEED = MOVEMENT_SPEED
ROTATION_SPEED = 0.6


# Algorithms info
WAIT_TICKS_ON_SONAR = 4
WAIT_TICKS_ON_FORWARD = 15

SHOT_ANGLE_ON_OBSTACLE = 20
MAX_ANGLE_TO_CHECK_ON_OBSTACLE = 75

ROTATE_ON_AVOIDING = 10

MAX_MISSING_ANGLE = 15


# Dictionary reading
ANGLES = "ANGLES"
FINISH = "FINISH"


# Info saving
CURRENT_ID = "CURRENT_ID"
CURRENT_DIRECTION = "CURRENT_DIRECTION"
CURRENT_ANGLE = "CURRENT_ANGLE"
READ_IDS = "READ_IDS"
STATE = "STATE"

ERROR_MESSAGE = "ERROR_MESSAGE"
IS_IN_MAIN = "IS_IN_MAIN"

