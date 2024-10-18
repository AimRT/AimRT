# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import os

# ANSI escape codes for coloring the output
RESET = "\033[0m"
BOLD = "\033[1m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
MAGENTA = "\033[35m"
CYAN = "\033[36m"
WHITE = "\033[37m"
BRIGHT_GREEN = "\033[92m"


# Test result codes
class TestResult:
    SUCCESS = 0
    EXPECTED_OUTPUT_NOT_FOUND = 1
    FORBIDDEN_OUTPUT_FOUND = 2
    EXIT_STRING_NOT_FOUND = 3


# Common forbiden outputs for all modules
default_forbidden_outputs = [
    "[Error]",
    "[Fatal]",
    "Segmentation fault" "core dumped",
    "Traceback (most recent call last):",
]

# Default expected outputs for pb_chn_cpp
default_pb_chn_sub_expected_outputs_cpp = ['data: {"msg":"count: 1","num":1}']
default_pb_chn_pub_expected_outputs_cpp = ['Publish new pb event, data: {"msg":"count: 1","num":1}']

# Default expected outputs for pb_rpc_cpp
default_pb_rpc_srv_expected_outputs_cpp = [
    'req: {"msg":"hello world foo, count 1"}, return rsp: {"code":"0","msg":"echo hello world foo, count 1"}'
]
default_pb_rpc_cli_expected_outputs_cpp = [
    'req: {"msg":"hello world foo, count 1"}',
    'rsp: {"code":"0","msg":"echo hello world foo, count 1"}',
]

# Default expected outputs for ros2_chn_cpp
default_ros2_chn_sub_expected_outputs_cpp = ["Receive new ros event, data:", "test_msg2"]
default_ros2_chn_pub_expected_outputs_cpp = ["Publish new ros event, data:", "test_msg2:"]

# Default expected outputs for ros2_rpc_cpp
default_ros2_rpc_srv_expected_outputs_cpp = ["Get new rpc call. context: Server context", "Svr rpc time cost"]
default_ros2_rpc_cli_expected_outputs_cpp = [
    "start new rpc call. req:",
    "Get rpc ret, status: suc, code 0, msg: OK. rsp:",
]

# Default expected outputs for pb_chn_python
default_pb_chn_sub_expected_outputs_python = ["Get new pb event, data: {", '"msg": "example msg"', '"num": 123456']
default_pb_chn_pub_expected_outputs_python = ["Publish new pb event, data: {", '"msg": "example msg"', '"num": 123456']


# Default expected outputs for pb_rpc_python
default_pb_rpc_srv_expected_outputs_python = ["Server handle new rpc call."]
default_pb_rpc_cli_expected_outputs_python = ["Call rpc done, status: suc, code 0, msg: OK"]

# Default exit string
default_exit_string = "AimRT exit."

# Default timeout, unit: second
default_timeout = 4


# Todo this is not the best way to find the aim directory, need to improve it
def upwards_find_aim_directory(aim: str = "build", start_directory: str = os.getcwd()) -> str:
    current_directory = start_directory

    # try to find the aim directory in the current directory
    while True:
        aim_directory = os.path.join(current_directory, aim)

        if os.path.isdir(aim_directory):
            return aim_directory

        # move to the parent directory
        parent_directory = os.path.dirname(current_directory)

        # if come to the root directory, stop searching
        if parent_directory == current_directory:
            break
        current_directory = parent_directory

    assert False, f"Cannot find {aim} directory."


# Default build path
defualt_build_path = upwards_find_aim_directory()

# Default py_path
py_cwd = defualt_build_path + "/../src/examples/py"

default_save_path = os.path.join(os.getcwd(), "test_log")
