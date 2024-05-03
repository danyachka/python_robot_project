import math
import time

from src import constants
from src.constants import gyro_dt


class GyroscopeEmulator:
    def __init__(self):
        self.prev_angles = [0, 0, 0]
        self.timestamp = time.time()

    def update(self, current_angles):
        current_time = time.time()
        # dt = current_time - self.timestamp
        dt = constants.gyro_dt
        if dt == 0:
            return [0, 0, 0]

        delta_angles = [current_angles[i] - self.prev_angles[i] for i in range(3)]

        gyro_data = [delta_angle * 180.0 / (dt * math.pi) for delta_angle in delta_angles]

        # Обновляем предыдущие углы и временную метку
        self.prev_angles = current_angles
        self.timestamp = current_time

        return gyro_data
