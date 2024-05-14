import math

main_dt = 0.1

gyro_dt = 1 / 100


mainThreadId = 0
gyroThreadId = 1


SONAR_MAX_DIST = 5
SONAR_FAKE_MIN = 0.04


ARUCO_DISTANCE = 0.6


MOVEMENT_SPEED = 2
LOW_MOVEMENT_SPEED = MOVEMENT_SPEED


imageW = 640
imageH = 480
CAMERA_ANGLE = math.pi / 4
