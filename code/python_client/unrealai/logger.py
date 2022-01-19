import logging
import sys, os
from typing import Optional
from unrealai.constants import PACKAGE_ROOT_DIR

LOG_FORMAT = "%(asctime)s %(levelname)s [%(filename)s:%(lineno)d] %(message)s"

_loggers = set()


def get_logger(
    name: str,
    handler_type: Optional[str],
    filename=os.path.join(PACKAGE_ROOT_DIR, "logs", "unrealai-log.txt"),
) -> logging.Logger:
    """
    Creates a logger with the specified name. The logger will use the log level specified by set_log_level().
    Also, handler can be file/stream, depending on input param @handler_type
    """

    logger = logging.getLogger(name=name)

    # create dir if it does not exist
    if not os.path.exists(os.path.dirname(filename)):
        os.mkdir(os.path.dirname(filename))

    # create an empty log file
    if not os.path.exists(filename):
        with open(filename, "w") as f:
            pass

    # create formatter based on LOG_FORMAT
    formatter = logging.Formatter(fmt=LOG_FORMAT)

    # create handlers
    handlers = set()
    if handler_type == "file":
        handlers.add(logging.FileHandler(filename=filename))
        handlers.add(logging.StreamHandler(stream=sys.stdout))
    else:
        handlers.add(logging.StreamHandler(stream=sys.stdout))

    for handler in handlers:
        handler.setFormatter(formatter)
        logger.addHandler(handler)

    # If we've already set the log level, make sure new loggers use it
    logger.setLevel(logging.INFO)

    # Keep track of this logger so that we can change the log level later
    _loggers.add(logger)
    return logger


def set_log_level(logger: logging.Logger, log_level: int) -> None:
    """
    Set the UnrealAI logging level. This will also configure the logging format (if it hasn't already been set).
    """

    logger.setLevel(log_level)

    formatter = logging.Formatter(fmt=LOG_FORMAT)

    _set_formatter_for_all_loggers(formatter)


def _set_formatter_for_all_loggers(formatter: logging.Formatter) -> None:
    for logger in _loggers:
        for handler in logger.handlers[:]:
            handler.setFormatter(formatter)
