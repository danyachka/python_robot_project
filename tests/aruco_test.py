from pathlib import Path
import cv2 as cv

from src.ananlysing_scripts.camera_script import ArucoDetector


def main():
    path = Path(__file__).parent.parent.joinpath("data/aruco.png")

    img = cv.imread(str(path))

    arucoDetector = ArucoDetector()

    arucoDetector.onImage(img)


if __name__ == '__main__':
    main()
