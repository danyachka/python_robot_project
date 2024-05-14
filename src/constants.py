import math

# Thread info
main_dt = 0.1

gyro_dt = 1 / 100

mainThreadId = 0
gyroThreadId = 1


# Sensors info
SONAR_MAX_DIST = 5
SONAR_FAKE_MIN = 0.04

ARUCO_DISTANCE = 0.6

imageW = 640
imageH = 480
CAMERA_ANGLE = math.pi / 4


# Movement info
MOVEMENT_SPEED = 2
LOW_MOVEMENT_SPEED = MOVEMENT_SPEED

MAX_MISSING_ANGLE = 8
