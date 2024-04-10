import math
import time


class GyroscopeEmulator:
    def __init__(self):
        self.prev_angles = [0, 0, 0]  # Предыдущие углы поворота по осям X, Y, Z
        self.timestamp = time.time()  # Текущее время

    def update(self, current_angles):
        # Получаем текущее время
        current_time = time.time()
        # Вычисляем разницу времени с предыдущим обновлением
        dt = current_time - self.timestamp

        # Вычисляем изменение углов поворота
        delta_angles = [current_angles[i] - self.prev_angles[i] for i in range(3)]

        # Вычисляем угловую скорость как изменение угла на секунду
        gyro_data = [delta_angle * 180.0 / (dt * math.pi) for delta_angle in delta_angles]

        # Обновляем предыдущие углы и временную метку
        self.prev_angles = current_angles
        self.timestamp = current_time

        return gyro_data
