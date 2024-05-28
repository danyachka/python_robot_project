import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt


def __initializeKalmanFilter():
    kf = cv.KalmanFilter(2, 1)
    kf.measurementMatrix = np.array([[1, 0]], np.float32)
    kf.transitionMatrix = np.array([[1, 1],
                                    [0, 1]], np.float32)
    kf.processNoiseCov = np.eye(2, dtype=np.float32) * 1e-4
    kf.measurementNoiseCov = np.eye(1, dtype=np.float32) * 1e-1
    kf.errorCovPost = np.eye(2, dtype=np.float32) * 0.1
    kf.statePost = np.zeros((2, 1), np.float32)

    return kf


def readGyro(kf, dt, measurement) -> float:

    kf.transitionMatrix[0, 1] = dt
    kf.correct(measurement)
    prediction = kf.predict()

    return prediction


if __name__ == '__main__':
    kalman = __initializeKalmanFilter()

    dt = 0.01
    l = 1000
    rand = np.random.uniform(low=0.5, high=20, size=(l,))
    data = np.array([5 for _ in range(l)], np.float32) + rand

    result = np.zeros_like(data)
    result_2 = np.zeros_like(data)

    for i in range(len(data)):
        print(data[i])
        measurement = data[i]
        result[i] = result[i - 1] + readGyro(kalman,
                                             dt,
                                             np.array(measurement, np.float32))[1]

        result_2[i] = result_2[i - 1] + dt * measurement

    plt.figure(figsize=(10, 6))
    plt.plot(result_2, "-D", color="blue")
    plt.plot(result, "-D", color="green")
    plt.title("Blue hist")

    plt.show()
