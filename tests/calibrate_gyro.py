from mpu6050 import mpu6050
import time

imu = mpu6050(0x68)


def get_gyro():
    data = imu.get_gyro_data()
    gx = data['x']
    gy = data['y']
    gz = data['z']
    return gx, gy, gz


def gyro_calibration(calibration_time=10):
    print('--' * 25)
    print('Beginning Gyro Calibration - Do not move the MPU6050')

    offsets = [0, 0, 0]
    num_of_points = 0

    end_loop_time = time.time() + calibration_time
    while end_loop_time > time.time():
        num_of_points += 1
        (gx, gy, gz) = get_gyro()
        offsets[0] += gx
        offsets[1] += gy
        offsets[2] += gz

        if num_of_points % 100 == 0:
            print('Still Calibrating Gyro... %d points so far' % num_of_points)

    print('Calibration for Gyro is Complete! %d points total' % num_of_points)
    offsets = [i / num_of_points for i in offsets]
    return offsets


if __name__ == '__main__':
    res = gyro_calibration(10)

    print(f'result = {res}')
