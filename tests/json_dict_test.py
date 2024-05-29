import json
from src import constants
from pathlib import Path


def createJson():
    toJson = {
        constants.IS_EMULATION: True,
        constants.PRINT_ONLY_IMPORTANT_LOGS: True,
        constants.BUILD_PLOT: False
    }

    print(json.dumps(toJson))


def loadJson():
    p = Path(__file__).parent.parent.joinpath("data").joinpath("configuration.json")
    file = open(str(p), "r")

    text = file.read()

    d = json.loads(text)
    print(d)

    # finish = d[constants.FINISH]
    # print(finish)
    #
    # angles = {}
    # anglesJson = d[constants.ANGLES]
    # for key in anglesJson.keys():
    #     angles[int(key)] = anglesJson[key]
    #
    # print(angles)


if __name__ == "__main__":
    loadJson()
