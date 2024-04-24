
iterationCount = 0

printRate = 20


def logRated(text, tag=""):

    if iterationCount % printRate == 0:
        log(text, tag)


def log(text, tag=""):
    print(f"{tag:20}: {text}")


def setStep():
    global iterationCount
    iterationCount += 1
