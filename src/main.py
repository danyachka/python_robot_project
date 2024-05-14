from execution_scripts.iterator import Iterator


def parseDictionary() -> tuple[dict[int, float], int]:
    return {
        0: -84.62,
        20: 79.46,
        15: 8,
        6: 2
    }, 13


def main():
    d, f = parseDictionary()
    iterator = Iterator(True, d, f)
    iterator.start()


if __name__ == '__main__':
    main()
