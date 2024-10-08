import math

import cv2

# Thread info
MAIN_FR = 10
main_dt = 1 / MAIN_FR

GYRO_FR = 100
gyro_dt = 1 / GYRO_FR

mainThreadId = 0
gyroThreadId = 1


# Physics
SOUND_SPEED = 343.00


# Size info (from config)
ARUCO_SIDE_SIZE = 40  # sm
ARUCO_SIDE_SIZE_TO_METRIC = 100  # sm


# Sensors info
SONAR_DIST_NOTHING = 5_000
SONAR_MAX_DIST = 3
SONAR_FAKE_MIN = 0.02

SONAR_MAX_SIGNAL_WAITING = main_dt * 0.9
SONAR_MAX_DURATION = SONAR_MAX_DIST * 2 / SOUND_SPEED
SONAR_CHECK_ITERATION_SLEEP = 0.0001

ARUCO_DISTANCE = 0.8
ARUCO_DISTANCE_CHECKING = 1.5
ARUCO_DISTANCE_PARKING = ARUCO_DISTANCE_CHECKING * 2
ARUCO_CAMERA_DISTANCE = 3

OBSTACLE_DISTANCE = 1.4
OBSTACLE_SUBTRACT_DISTANCE = 0.6
OBSTACLE_SIDE_DISTANCE = 0.6
OBSTACLE_BACK_DISTANCE = 0.25

MAX_BOTTOM_SCANNER_DATA = 0.15

imageW = 640
imageH = 480
CAMERA_ANGLE = 45 * math.pi / 180


# Movement info
MOVEMENT_SPEED = 8
LOW_MOVEMENT_SPEED = 6
ROTATION_SPEED = 0.7


# Algorithms info
WAIT_TICKS_ON_ROTATION = round(MAIN_FR * 0.5)

WAIT_TICKS_ON_SONAR = round(MAIN_FR / 4)
WAIT_TICKS_ON_FORWARD = round(MAIN_FR / 1)

WAIT_TICKS_ON_PARKING = round(MAIN_FR * 4)
WAIT_TICKS_ON_PARKING_FINALLY_ENDED = round(MAIN_FR * 3)
WAIT_TICKS_ON_PARKING_END_CLOSER = round(MAIN_FR * 1)
WAIT_TICKS_ON_PARKING_KEEP_MOVING = round(MAIN_FR * 0.7)
WAIT_TICKS_ON_PARKING_END = round(MAIN_FR)

WAIT_TICKS_ON_GOING_BACK_PIT = round(MAIN_FR * 2)

SHOT_ANGLE_ON_OBSTACLE = 20
MAX_ANGLE_TO_CHECK_ON_OBSTACLE = 75

ROTATE_ON_AVOIDING = 15

MAX_MISSING_ANGLE = 7

MAX_MISSING_ANGLE_ON_MOVING = 10

MISSING_ANGLE_PARKING = 15


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


# configuration
IS_EMULATION = "IS_EMULATION"
PRINT_ONLY_IMPORTANT_LOGS = "PRINT_ONLY_IMPORTANT_LOGS"
BUILD_PLOT = "BUILD_PLOT"
USE_FLASK = "USE_FLASK"
FLASK_DELAY = "FLASK_DELAY"

ARUCO_DICTIONARY = "ARUCO_DICTIONARY"
ARUCO_SIZE = "ARUCO_SIZE"


# configuration (Values will be replaced by config)
isEmulation: bool = False
onlyImportantLogs: bool = True
buildPlot: bool = False
use_flask: bool = False
flask_delay: float = 0.2
arucoDictionary: int = cv2.aruco.DICT_4X4_1000
