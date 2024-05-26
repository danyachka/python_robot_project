from execution_scripts.iterator import Iterator
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


def main():
    d, f = parseDictionary()

    isEmulation = True
    onlyImportantLogs = True

    iterator = Iterator(True, d, f)
    logger.onlyImportant = onlyImportantLogs
    iterator.start()


if __name__ == '__main__':
    main()
