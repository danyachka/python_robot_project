import threading

from colorama import Fore

from src import constants


onlyImportant = False


def __getTheadName():
    threadId = threading.get_ident()

    if threadId == constants.mainThreadId:
        return "Main "
    elif threadId == constants.gyroThreadId:
        return "Gyro "

    return Fore.RED + "Unkn " + Fore.RESET


def __deny(isImportant) -> bool:
    if not onlyImportant:
        return False

    return not isImportant


def log(text, tag="", isImportant=False):
    if __deny(isImportant):
        return
    print(f"{__getTheadName()}{tag:20}: {text}")


def logBlue(text, tag="", isImportant=False):
    if __deny(isImportant):
        return
    print(f"{__getTheadName()}{tag:20}: {Fore.BLUE}{text}{Fore.RESET}")


def logError(text, tag=""):
    print(f"{__getTheadName()}{tag:20}: {Fore.YELLOW}{text}{Fore.RESET}")


def logCriticalError(text, tag=""):
    print(f"{__getTheadName()}{tag:20}: {Fore.RED}{text}{Fore.RESET}")
