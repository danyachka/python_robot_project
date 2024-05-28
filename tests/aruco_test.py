from pathlib import Path
import cv2 as cv
import numpy as np

from src.ananlysing_scripts.camera_script import ArucoDetector


def emuTest(path):
    cameraMatrix = np.array([[580.1665156, 0., 325.59939736],
                             [0., 579.40992364, 232.22784327],
                             [0., 0., 1.]])

    distCfs = np.array([[-4.62821723e-01, 5.81112136e-01, 2.66871651e-03, 3.02822923e-04, -8.95088570e-01]])

    arucoDetector = ArucoDetector(cameraMatrix, distCfs, True)

    for i in range(4):
        prefix = 'img.png'
        if i == 1:
            prefix = 'img_1.png'
        elif i == 2:
            prefix = 'img_2.png'
        elif i == 3:
            prefix = "aruco.png"

        img = cv.imread(str(path.joinpath(prefix)))

        arucoDetector.onImage(img)


def realTest(path):
    path = path.joinpath("real_aruco")

    count = int(input('count?'))
    cameraMatrix = np.array([[582.86635316, 0., 321.49076078],
                             [0., 584.82488533, 234.52954564],
                             [0., 0., 1.]])

    distCfs = np.array([[-0.39671064, 0.0474044, 0.00244292, -0.00081249, 0.56456562]])

    arucoDetector = ArucoDetector(cameraMatrix, distCfs, True)

    for i in range(1, count + 1):
        prefix = f"{i}.jpg"
        p = str(path.joinpath(prefix))
        print(p)

        img = cv.imread(p)

        arucoDetector.onImage(img)


def main():
    path = Path(__file__).parent.parent.joinpath("data")

    if input("emu/real? (e/r)") == 'e':
        emuTest(path)
    else:
        realTest(path)


if __name__ == '__main__':
    main()
