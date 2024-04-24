from colorama import Fore


def log(text, tag=""):
    print(f"{tag:20}: {text}")


def logBlue(text, tag=""):
    print(f"{tag:20}: {Fore.BLUE}{text}{Fore.RESET}")


def logError(text, tag=""):
    print(f"{tag:20}: {Fore.YELLOW}{text}{Fore.RESET}")
