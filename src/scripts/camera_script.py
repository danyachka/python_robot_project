from PIL import Image
import numpy as np
import cv2 as cv


def onImage(image: np.ndarray):
    if image is None:
        return

    cv.imshow("Camera image", image)
    cv.waitKey(1)
