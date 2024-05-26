import math
import time

from src import constants
from src.logger import log, logError


last_dt = constants.gyro_dt


class GyroscopeEmulator:
    def __init__(self):
        self.prev_angles = [0, 0, 0]
        self.timestamp = 0

    def update(self, current_angles):
        current_time = time.time()

        dt = last_dt
        if dt == 0 or self.timestamp == 0:
            dt = constants.gyro_dt

        if abs(dt - last_dt) > 0.001:
            logError(f"Gyro emulator dt = {1/dt}, but analyser dt = {1/last_dt}", self.__class__.__name__)
        if dt == 0:
            return [0, 0, 0]

        delta_angles = [current_angles[i] - self.prev_angles[i] for i in range(3)]

        gyro_data = [delta_angle * 180.0 / (dt * math.pi) for delta_angle in delta_angles]

        self.prev_angles = current_angles
        self.timestamp = current_time

        return gyro_data
