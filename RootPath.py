import os


def get_root() -> str:
    """
    :return: Root path of the project
    """
    return os.path.dirname(os.path.abspath(__file__))
