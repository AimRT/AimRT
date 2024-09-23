# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import inspect


def trace(logger, msg):
    if 0 >= logger.GetLogLevel():
        info = inspect.getframeinfo(inspect.currentframe().f_back)
        logger.Log(0, info.lineno, 0, info.filename, info.function, msg, len(msg))


def debug(logger, msg):
    if 1 >= logger.GetLogLevel():
        info = inspect.getframeinfo(inspect.currentframe().f_back)
        logger.Log(1, info.lineno, 0, info.filename, info.function, msg, len(msg))


def info(logger, msg):
    if 2 >= logger.GetLogLevel():
        info = inspect.getframeinfo(inspect.currentframe().f_back)
        logger.Log(2, info.lineno, 0, info.filename, info.function, msg, len(msg))


def warn(logger, msg):
    if 3 >= logger.GetLogLevel():
        info = inspect.getframeinfo(inspect.currentframe().f_back)
        logger.Log(3, info.lineno, 0, info.filename, info.function, msg, len(msg))


def error(logger, msg):
    if 4 >= logger.GetLogLevel():
        info = inspect.getframeinfo(inspect.currentframe().f_back)
        logger.Log(4, info.lineno, 0, info.filename, info.function, msg, len(msg))


def fatal(logger, msg):
    if 5 >= logger.GetLogLevel():
        info = inspect.getframeinfo(inspect.currentframe().f_back)
        logger.Log(5, info.lineno, 0, info.filename, info.function, msg, len(msg))
