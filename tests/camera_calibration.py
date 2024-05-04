from pathlib import Path
import cv2 as cv
import numpy as np


def main():
    answer = input("emulation/real (e/r)?")
    image_count = int(input("Image count?"))

    path: Path
    if answer == "e":
        path = Path(__file__).parent.parent.joinpath("data/chess_emulation/")
    elif answer == "r":
        path = Path(__file__).parent.parent.joinpath("data/chess_real/")
    else:
        raise Exception("Answer valur should be e or m")

    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6 * 7, 3), np.float32)
    objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objPoints = []  # 3d point in real world space
    imgPoints = []  # 2d points in image plane.

    for i in range(1, image_count + 1):
        fileName = str(path) + f'{i}.png'
        img = cv.imread(fileName)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (7, 6), None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objPoints.append(objp)

        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgPoints.append(corners2)

        # Draw and display the corners
        cv.drawChessboardCorners(img, (7, 6), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)

    cv.destroyAllWindows()

    _, cameraMatrix, distCoeffs, _, _ = cv.calibrateCamera(objPoints, imgPoints, gray.shape[::-1], None, None)

    print(f"Camera Matrix:\n{cameraMatrix}\ndistCoeffs:\n{distCoeffs}")


if __name__ == '__main__':
    main()
