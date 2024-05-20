import math

# Thread info
main_dt = 0.1

gyro_dt = 0.01

mainThreadId = 0
gyroThreadId = 1


# Sensors info
SONAR_MAX_DIST = 5
SONAR_FAKE_MIN = 0.04

ARUCO_DISTANCE = 0.6
OBSTACLE_DISTANCE = 2.4

imageW = 640
imageH = 480
CAMERA_ANGLE = 25 * math.pi / 180


# Movement info
MOVEMENT_SPEED = 2
LOW_MOVEMENT_SPEED = MOVEMENT_SPEED

MAX_MISSING_ANGLE = 8
