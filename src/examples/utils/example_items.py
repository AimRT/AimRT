# this is a file to store the items for the example
# fotmat:
#     ·script_path*          # Required: Specifies the path to the script that will be executed.
#     ·expected_outputs*     # Required: The expected output result of the test.
#     ·tags                  # recommended: Tags for categorizing the test item, facilitating management and retrieval.
#     ·forbidden_outputs     # Optional: Specifies output results that are not allowed.
#     ·exit_string           # Optional: Specifies the string that the script should output to indicate successful execution.
#     ·timeout               # Optional: Sets a timeout duration for the test execution to prevent it from hanging indefinitely.
#     ·cwd                   # Optional: Specifies the current working directory, used as context when running the script.
#     ·limits                # Optional: Sets restrictions for executing the script, such as memory or time limit.
#     ·priority              # Optional: Specifies the priority of the test item for better organization during test execution.
#     ·dependencies:         # Optional: Lists other components or libraries that the test item depends on.
#     ·description:          # Optional: A detailed description of the test item, explaining its purpose and background.
#     ·expansion:            # Optional: Indicates any planned or potential expansion related to the test item.

import sys, os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from utils.common import *

# todo auto locate
common_timeout1 = 3
common_timeout2 = 4

cpp_items = [
    {
        "script_path": [
            "./start_examples_plugins_mqtt_plugin_pb_chn_sub.sh",
            "./start_examples_plugins_mqtt_plugin_pb_chn_pub.sh",
        ],
        "expected_outputs": [
            default_pb_chn_sub_expected_outputs,
            default_pb_chn_pub_expected_outputs,
        ],
        "tags": ["all", "plugins", "mqtt", "pb", "chn"],
    },
    {
        "script_path": [
            "./start_examples_cpp_executor.sh",
        ],
        "expected_outputs": [
            ["executor"],
        ],
        "tags": ["all", "cpp", "executor"],
    },
    {
        "script_path": [
            "./iox-roudi",
            "./start_examples_plugins_iceoryx_plugin_pb_chn_sub.sh",
            "./start_examples_plugins_iceoryx_plugin_pb_chn_pub.sh",
        ],
        "expected_outputs": [
            ["ready"],
            default_pb_chn_sub_expected_outputs,
            default_pb_chn_pub_expected_outputs,
        ],
        "exit_string": [
            "",
            default_exit_string,
            default_exit_string,
        ],
        "tags": ["all", "plugins", "iceoryx", "pb", "chn"],
    },
    {
        "script_path": [
            "./start_examples_plugins_log_control_plugin.sh",
            "./tools/log_control_plugin_get_lvl.sh",
        ],
        "expected_outputs": [
            ["s = abc, n = 1"],
            ['{"code":0,"msg":"","module_log_level_map":'],
        ],
        "forbidden_outputs": [
            "",
            default_forbidden_outputs,
        ],
        "exit_string": [
            default_exit_string,
            "",
        ],
        "tags": ["all", "plugins", "log_control", "curl"],
        "limit": "port:50080",
    },
    {
        "script_path": [
            "./native_ros2_pb_chn_subscriber",
            "./start_examples_plugins_ros2_plugin_pb_chn_pub.sh",
        ],
        "expected_outputs": [
            ["msg: count: 1"],
            default_pb_chn_pub_expected_outputs,
        ],
        "exit_string": [
            "",
            default_exit_string,
        ],
        "tags": ["all", "ro2_plugin", "pb", "chn", "native"],
    },
    {
        "script_path": [
            "./start_examples_plugins_net_plugin_pb_chn_http_sub.sh",
            "./start_examples_plugins_net_plugin_pb_chn_http_pub.sh",
        ],
        "expected_outputs": [
            default_pb_chn_sub_expected_outputs,
            default_pb_chn_pub_expected_outputs,
        ],
        "tags": ["all", "net", "http", "pb", "chn"],
        "limit": "port:50080",
    },
]
