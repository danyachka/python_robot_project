import json
from src import constants
from pathlib import Path


def createJson():
    toJson = {
        constants.ANGLES: {
            0: -84.62,
            20: 79.46,
            15: 8,
            6: 2
        },
        constants.FINISH: 13
    }

    print(json.dumps(toJson))


if __name__ == "__main__":
    p = Path(__file__).parent.parent.joinpath("data").joinpath("dictionary.json")
    file = open(str(p), "r")

    text = file.read()

    d = json.loads(text)

    finish = d[constants.FINISH]
    print(finish)

    angles = {}
    anglesJson = d[constants.ANGLES]
    for key in anglesJson.keys():
        angles[int(key)] = anglesJson[key]

    print(angles)
