from execution_scripts.iterator import Iterator


def parseDictionary() -> dict[int, float]:
    return {
        0: -84.62,
        20: 79.46,
        15: 8,
        6: 21.9
    }


def main():
    iterator = Iterator(True, parseDictionary())
    iterator.start()


if __name__ == '__main__':
    main()
