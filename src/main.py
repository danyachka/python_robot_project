import math
import sys
from pathlib import Path
import json
from threading import Thread

import cv2.aruco

from src.execution_scripts.stream import stream

from src import constants
from src import logger

import keyboard


def parseDictionary() -> tuple[dict[int, float], int]:
    path = Path(__file__).parent.parent.joinpath("data").joinpath("dictionary.json")
    file = open(str(path), "r")
    text = file.read()

    jsonDict = json.loads(text)

    finish = jsonDict[constants.FINISH]

    angles = {}
    anglesJson = jsonDict[constants.ANGLES]
    for key in anglesJson.keys():
        angles[int(key)] = anglesJson[key]

    logger.logBlue(f"id: angles = {angles}")
    logger.logBlue(f"Finish id = {finish}")
    return angles, finish


def parseConfiguration():

    path = Path(__file__).parent.parent.joinpath("data").joinpath("configuration.json")
    file = open(str(path), "r")
    text = file.read()

    d = json.loads(text)

    constants.isEmulation = d[constants.IS_EMULATION]
    if not constants.isEmulation:
        constants.MOVEMENT_SPEED = 50
        constants.LOW_MOVEMENT_SPEED = 40
        constants.ROTATION_SPEED = 40
        constants.CAMERA_ANGLE = 67 * math.pi / 180

    constants.onlyImportantLogs = d[constants.PRINT_ONLY_IMPORTANT_LOGS]
    constants.buildPlot = d[constants.BUILD_PLOT]
    constants.use_flask = d[constants.USE_FLASK]
    constants.flask_delay = d[constants.FLASK_DELAY]

    constants.ARUCO_SIDE_SIZE = d[constants.ARUCO_SIZE]
    arucoDict = d[constants.ARUCO_DICTIONARY]

    match arucoDict:
        case "DICT_4X4_50":
            constants.arucoDictionary = cv2.aruco.DICT_4X4_50
        case "DICT_4X4_100":
            constants.arucoDictionary = cv2.aruco.DICT_4X4_100
        case "DICT_4X4_250":
            constants.arucoDictionary = cv2.aruco.DICT_4X4_250
        case "DICT_4X4_1000":
            constants.arucoDictionary = cv2.aruco.DICT_4X4_1000

        case "DICT_5X5_50":
            constants.arucoDictionary = cv2.aruco.DICT_5X5_50
        case "DICT_5X5_100":
            constants.arucoDictionary = cv2.aruco.DICT_5X5_100
        case "DICT_5X5_250":
            constants.arucoDictionary = cv2.aruco.DICT_5X5_250
        case "DICT_5X5_1000":
            constants.arucoDictionary = cv2.aruco.DICT_5X5_1000

        case "DICT_6X6_50":
            constants.arucoDictionary = cv2.aruco.DICT_6X6_50
        case "DICT_6X6_100":
            constants.arucoDictionary = cv2.aruco.DICT_6X6_100
        case "DICT_6X6_250":
            constants.arucoDictionary = cv2.aruco.DICT_6X6_250
        case "DICT_6X6_1000":
            constants.arucoDictionary = cv2.aruco.DICT_6X6_1000

        case "DICT_7X7_50":
            constants.arucoDictionary = cv2.aruco.DICT_7X7_50
        case "DICT_7X7_100":
            constants.arucoDictionary = cv2.aruco.DICT_7X7_100
        case "DICT_7X7_250":
            constants.arucoDictionary = cv2.aruco.DICT_7X7_250
        case "DICT_7X7_1000":
            constants.arucoDictionary = cv2.aruco.DICT_7X7_1000


def main():
    d, f = parseDictionary()

    parseConfiguration()

    from src.execution_scripts.iterator import Iterator

    iterator = Iterator(d, f)
    logger.onlyImportant = constants.onlyImportantLogs

    keyboard.add_hotkey("ctrl+alt+p", lambda: onKeyboardInterrupt(iterator))

    if constants.use_flask:
        thread = Thread(target=iterator.start, daemon=True)
        thread.start()

        try:
            stream.delay = constants.flask_delay
            stream.start()
        except Exception as e:
            iterator.onDestroy()
            iterator.onException(e, True)
    else:
        stream.app = None

        iterator.start()


def onKeyboardInterrupt(iterator):
    iterator.onDestroy()
    print("System's been released")
    sys.exit("Penis movement's been detected")


if __name__ == '__main__':
    main()
