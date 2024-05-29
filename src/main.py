from pathlib import Path
import json
from threading import Thread

from src.execution_scripts.stream import stream

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
    constants.use_flask = d[constants.USE_FLASK]
    constants.flask_delay = d[constants.FLASK_DELAY]


def main():
    d, f = parseDictionary()

    parseConfiguration()

    from src.execution_scripts.iterator import Iterator

    iterator = Iterator(d, f)
    logger.onlyImportant = constants.onlyImportantLogs

    if constants.use_flask:

        thread = Thread(target=iterator.start, daemon=True)
        thread.start()

        stream.delay = constants.flask_delay
        stream.start()
    else:
        stream.app = None

        iterator.start()


if __name__ == '__main__':
    main()
