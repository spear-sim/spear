from . import logger

logger = logger.get_logger(__name__, "file")


class UnrealException(Exception):
    """
    Any error related to Unreal Server and client setup.
    """

    def __init__(self, message):
        self.message = message
        super().__init__(self.message)
        logger.error(f"{self.message}")

    pass


class UnrealPlaformException(Exception):
    """
    Errors related to running on different platforms.
    """

    pass


class UnrealEnvException(UnrealException):
    """
    Any error related to Unreal Environment.
    """

    pass


class UnrealObservationException(UnrealException):
    """
    Related to errors with receiving observations.
    """

    pass


class UnrealActionException(UnrealException):
    """
    Related to errors with sending actions.
    """

    pass


class UnrealConfigError(UnrealException):
    """
    Raise error related to reading config file.
    """

    pass


class UnrealActionSpecException(UnrealException):
    """
    Related to errors w.r.t action specifications.
    """

    pass


class UnrealObservationSpecException(UnrealException):
    """
    Related to errors w.r.t observation specifications.
    """

    pass
