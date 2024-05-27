import time
from pathlib import Path
import cv2 as cv
import numpy as np


def main():
    answer = input("emulation/real (e/r)?\n")
    image_count = int(input("Image count?\n"))

    path: Path
    if answer == "e":
        path = Path(__file__).parent.parent.joinpath("data/chess_emulation/")
    elif answer == "r":
        path = Path(__file__).parent.parent.joinpath("data/chess_real/")
    else:
        raise Exception("Answer valur should be e or m")

    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    pattern_x = 5
    pattern_y = 5

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((pattern_x * pattern_y, 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_x, 0:pattern_y].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objPoints = []  # 3d point in real world space
    imgPoints = []  # 2d points in image plane.

    for i in range(1, image_count + 1):
        fileName = path.absolute().joinpath(f'{i}.jpg')
        print(str(fileName))
        image = cv.imread(str(fileName))
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        # gray = image

        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (pattern_x, pattern_y), None)

        # If found, add object points, image points (after refining them)
        if ret:
            objPoints.append(objp)

            corners2 = cv.cornerSubPix(gray, corners, (13, 13), (-1, -1), criteria)
            imgPoints.append(corners2)

            # Draw and display the corners
            cv.drawChessboardCorners(image, (pattern_x, pattern_y), corners2, ret)

        cv.imshow('img', image)
        cv.waitKey(500)

        time.sleep(0.4)

    cv.destroyAllWindows()

    _, cameraMatrix, distCoeffs, _, _ = cv.calibrateCamera(objPoints, imgPoints, gray.shape[::-1], None, None)

    print(f"Camera Matrix:\n{cameraMatrix}\ndistCoeffs:\n{distCoeffs}")


# last emulation

# Camera Matrix:
# [[632.87922628   0.         344.39433092]
#  [  0.         632.17162695 238.55829088]
#  [  0.           0.           1.        ]]
# distCoeffs:
# [[-3.25828939e-02  3.81686555e-01 -9.99220458e-04 -6.26907863e-04
#   -1.59300310e+00]]


if __name__ == '__main__':
    main()
