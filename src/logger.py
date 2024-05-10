import threading

from colorama import Fore

from src import constants


def __getTheadName():
    threadId = threading.get_ident()

    if threadId == constants.mainThreadId:
        return "Main "
    elif threadId == constants.gyroThreadId:
        return "Gyro "

    return Fore.RED + "Unkn " + Fore.RESET


def log(text, tag=""):
    print(f"{__getTheadName()}{tag:20}: {text}")


def logBlue(text, tag=""):
    print(f"{__getTheadName()}{tag:20}: {Fore.BLUE}{text}{Fore.RESET}")


def logError(text, tag=""):
    print(f"{__getTheadName()}{tag:20}: {Fore.YELLOW}{text}{Fore.RESET}")
