import sys
import os

class ConsoleColor(object):
    GRAY = "\033[1;30m"
    RED = "\033[1;31m"
    GREEN = "\033[0;32m"
    YELLOW = "\033[1;33m"
    BLUE = "\033[1;34m"
    MAGENTA = "\033[0;35m"
    CYAN = "\033[1;36m"

    RESET = "\033[0;0m"
    BOLD = "\033[;1m"
    REVERSE = "\033[;7m"

    def __init__(self, cc_color):
        self.cc_color = cc_color

    def __enter__(self):
        sys.stdout.write(self.cc_color)

    def __exit__(self, *args):
        sys.stdout.write(ConsoleColor.RESET)
        sys.stdout.flush()

    @classmethod
    def init_colors(cls):
        term = False
        if os.environ.get("TERM", None) is not None:
            term = True
        elif os.environ.get("ConEmuANSI", None) is not None:
            term = True
        if not term:
            cls.GRAY = ""
            cls.RED = ""
            cls.GREEN = ""
            cls.YELLOW = ""
            cls.BLUE = ""
            cls.MAGENTA = ""
            cls.CYAN = ""

            cls.RESET = ""
            cls.BOLD = ""
            cls.REVERSE = ""


ConsoleColor.init_colors()


def prompt_yellow(*args):
    with ConsoleColor(ConsoleColor.YELLOW):
        print(*args)


def prompt_green(*args):
    with ConsoleColor(ConsoleColor.GREEN):
        print(*args)

def prompt_red(*args):
    with ConsoleColor(ConsoleColor.RED):
        print(*args)

def prompt_blue(*args):
    with ConsoleColor(ConsoleColor.BLUE):
        print(*args)

def prompt_cyan(*args):
    with ConsoleColor(ConsoleColor.CYAN):
        print(*args)

def prompt_magenta(*args):
    with ConsoleColor(ConsoleColor.MAGENTA):
        print(*args)
