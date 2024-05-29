from pathlib import Path
import json
from src import constants
from src import logger


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
    constants.onlyImportantLogs = d[constants.PRINT_ONLY_IMPORTANT_LOGS]
    constants.buildPlot = d[constants.BUILD_PLOT]


def main():
    d, f = parseDictionary()

    parseConfiguration()

    from src.execution_scripts.iterator import Iterator

    iterator = Iterator(constants.isEmulation, constants.isEmulation, d, f)
    logger.onlyImportant = constants.onlyImportantLogs
    iterator.start()


if __name__ == '__main__':
    main()
