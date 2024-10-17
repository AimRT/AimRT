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


# common forbiden outputs for all modules
default_forbidden_outputs = [
    "[Error]",
    "[Fatal]",
    "Segmentation fault" "core dumped",
    "Traceback (most recent call last):",
]

default_pb_chn_sub_expected_outputs = ['data: {"msg":"count: 1","num":1}']
default_pb_chn_pub_expected_outputs = ["Loop count : 1 --", 'Publish new pb event, data: {"msg":"count: 1","num":1}']
default_pb_rpc_srv_expected_outputs = []
default_pb_rpc_cli_expected_outputs = []

default_ros2_chn_sub_expected_outputs = []
default_ros2_chn_pub_expected_outputs = []
default_ros2_rpc_srv_expected_outputs = []
default_ros2_rpc_cli_expected_outputs = []


default_exit_string = "AimRT exit."

default_cwd = "/home/hj/Desktop/AimRT_hj/build"  # todo: the shell script's current working directory

default_timeout = 4  # seconds

default_save_path = os.path.join(os.getcwd(), "test_log")
