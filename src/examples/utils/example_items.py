# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.


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


from utils.common import *

# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))


test_items = [
    # //////////////////////////////////////////////////////////////////////////////////////////
    # ////////////////////////////////CPP EXAMPLES//////////////////////////////////////////////
    # //////////////////////////////////////////////////////////////////////////////////////////
    # ------------------------------iceoryx_pb_chn---------------------------------------------
    {
        "script_path": [
            "./iox-roudi --config-file=./cfg/roudi_config.toml",
            "./start_examples_plugins_iceoryx_plugin_pb_chn_sub.sh",
            "./start_examples_plugins_iceoryx_plugin_pb_chn_pub.sh",
        ],
        "expected_outputs": [
            ["ready"],
            default_pb_chn_sub_expected_outputs_cpp,
            default_pb_chn_pub_expected_outputs_cpp,
        ],
        "exit_string": [
            "",
            default_exit_string,
            default_exit_string,
        ],
        "tags": ["all", "cpp", "plugins", "iceoryx", "pb", "chn"],
        "limit": "iox-roudi",
    },
    # ------------------------------iceoryx_ros2_chn---------------------------------------------
    {
        "script_path": [
            "./iox-roudi --config-file=./cfg/roudi_config.toml",
            "./start_examples_plugins_iceoryx_plugin_ros2_chn_sub.sh",
            "./start_examples_plugins_iceoryx_plugin_ros2_chn_pub.sh",
        ],
        "expected_outputs": [
            ["ready"],
            default_ros2_chn_sub_expected_outputs_cpp,
            default_ros2_chn_pub_expected_outputs_cpp,
        ],
        "exit_string": [
            "",
            default_exit_string,
            default_exit_string,
        ],
        "tags": ["all", "cpp", "plugins", "iceoryx", "ros2", "chn"],
        "limit": "iox-roudi",
    },
    # -------------------------------log_control_plugin----------------------------------------
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
        "tags": ["all", "cpp", "plugins", "log_control", "curl"],
        "limit": "port:50080",
    },
    # -------------------------------------tcp_pb_chn----------------------------------------------
    {
        "script_path": [
            "./start_examples_plugins_net_plugin_pb_chn_tcp_sub.sh",
            "./start_examples_plugins_net_plugin_pb_chn_tcp_pub.sh",
        ],
        "expected_outputs": [
            default_pb_chn_sub_expected_outputs_cpp,
            default_pb_chn_pub_expected_outputs_cpp,
        ],
        "tags": ["all", "cpp", "plugins", "net", "tcp", "pb", "chn"],
    },
    # -------------------------------------udp_pb_chn----------------------------------------------
    {
        "script_path": [
            "./start_examples_plugins_net_plugin_pb_chn_udp_sub.sh",
            "./start_examples_plugins_net_plugin_pb_chn_udp_pub.sh",
        ],
        "expected_outputs": [
            default_pb_chn_sub_expected_outputs_cpp,
            default_pb_chn_pub_expected_outputs_cpp,
        ],
        "tags": ["all", "cpp", "plugins", "net", "udp", "pb", "chn"],
    },
    # -------------------------------------http_pb_chn----------------------------------------------
    {
        "script_path": [
            "./start_examples_plugins_net_plugin_pb_chn_http_sub.sh",
            "./start_examples_plugins_net_plugin_pb_chn_http_pub.sh",
        ],
        "expected_outputs": [
            default_pb_chn_sub_expected_outputs_cpp,
            default_pb_chn_pub_expected_outputs_cpp,
        ],
        "tags": ["all", "cpp", "plugins", "net", "http", "pb", "chn"],
        "limit": "port:50080",
    },
    # -------------------------------------http_ros2_chn----------------------------------------------
    {
        "script_path": [
            "./start_examples_plugins_net_plugin_ros2_chn_http_sub.sh",
            "./start_examples_plugins_net_plugin_ros2_chn_http_pub.sh",
        ],
        "expected_outputs": [
            default_ros2_chn_sub_expected_outputs_cpp,
            default_ros2_chn_pub_expected_outputs_cpp,
        ],
        "tags": ["all", "cpp", "plugins", "net", "http", "ros2", "chn"],
        "limit": "port:50080",
    },
    # -------------------------------------http_pb_rpc----------------------------------------------
    {
        "script_path": [
            "./start_examples_plugins_net_plugin_pb_rpc_http_server.sh",
            "./start_examples_plugins_net_plugin_pb_rpc_http_client.sh",
        ],
        "expected_outputs": [
            default_pb_rpc_srv_expected_outputs_cpp,
            default_pb_rpc_cli_expected_outputs_cpp,
        ],
        "tags": ["all", "cpp", "plugins", "net", "http", "pb", "rpc"],
        "limit": "port:50080",
    },
    # -------------------------------------http_ros2_rpc----------------------------------------------
    {
        "script_path": [
            "./start_examples_plugins_net_plugin_ros2_rpc_http_server.sh",
            "./start_examples_plugins_net_plugin_ros2_rpc_http_client.sh",
        ],
        "expected_outputs": [
            default_ros2_rpc_srv_expected_outputs_cpp,
            default_ros2_rpc_cli_expected_outputs_cpp,
        ],
        "tags": ["all", "cpp", "plugins", "net", "http", "ros2", "rpc"],
        "limit": "port:50080",
    },
    # ---------------------------------------zenoh_pb_chn--------------------------------------------
    {
        "script_path": [
            "./start_examples_plugins_zenoh_plugin_pb_chn_sub.sh",
            "./start_examples_plugins_zenoh_plugin_pb_chn_pub.sh",
        ],
        "expected_outputs": [
            default_pb_chn_sub_expected_outputs_cpp,
            default_pb_chn_pub_expected_outputs_cpp,
        ],
        "tags": ["all", "cpp", "plugins", "zenoh", "pb", "chn"],
    },
    # ---------------------------------------zenoh_ros2_chn------------------------------------------
    {
        "script_path": [
            "./start_examples_plugins_zenoh_plugin_ros2_chn_sub.sh",
            "./start_examples_plugins_zenoh_plugin_ros2_chn_pub.sh",
        ],
        "expected_outputs": [
            default_ros2_chn_sub_expected_outputs_cpp,
            default_ros2_chn_pub_expected_outputs_cpp,
        ],
        "tags": ["all", "cpp", "plugins", "zenoh", "ros2", "chn"],
    },
    # ----------------------------------------zenoh_pb_rpc-------------------------------------
    {
        "script_path": [
            "./start_examples_plugins_zenoh_plugin_pb_rpc_server.sh",
            "./start_examples_plugins_zenoh_plugin_pb_rpc_client.sh",
        ],
        "expected_outputs": [
            default_pb_rpc_srv_expected_outputs_cpp,
            default_pb_rpc_cli_expected_outputs_cpp,
        ],
        "tags": ["all", "cpp", "plugins", "zenoh", "pb", "rpc"],
    },
    # -----------------------------------------zenoh_ros2_rpc------------------------------------
    {
        "script_path": [
            "./start_examples_plugins_zenoh_plugin_ros2_rpc_server.sh",
            "./start_examples_plugins_zenoh_plugin_ros2_rpc_client.sh",
        ],
        "expected_outputs": [
            default_ros2_rpc_srv_expected_outputs_cpp,
            default_ros2_rpc_cli_expected_outputs_cpp,
        ],
        "tags": ["all", "cpp", "plugins", "zenoh", "ros2", "rpc"],
    },
    # ------------------------------------------mqtt_pb_chn-----------------------------------
    {
        "script_path": [
            "./start_examples_plugins_mqtt_plugin_pb_chn_sub.sh",
            "./start_examples_plugins_mqtt_plugin_pb_chn_pub.sh",
        ],
        "expected_outputs": [
            default_pb_chn_sub_expected_outputs_cpp,
            default_pb_chn_pub_expected_outputs_cpp,
        ],
        "tags": ["all", "cpp", "plugins", "mqtt", "pb", "chn"],
    },
    # -------------------------------------------mqtt_ros2_chn----------------------------------
    {
        "script_path": [
            "./start_examples_plugins_mqtt_plugin_ros2_chn_sub.sh",
            "./start_examples_plugins_mqtt_plugin_ros2_chn_pub.sh",
        ],
        "expected_outputs": [
            default_ros2_chn_sub_expected_outputs_cpp,
            default_ros2_chn_pub_expected_outputs_cpp,
        ],
        "tags": ["all", "cpp", "plugins", "mqtt", "ros2", "chn"],
    },
    # --------------------------------------------mqtt_pb_rpc---------------------------------
    {
        "script_path": [
            "./start_examples_plugins_mqtt_plugin_pb_rpc_server.sh",
            "./start_examples_plugins_mqtt_plugin_pb_rpc_client.sh",
        ],
        "expected_outputs": [
            default_pb_rpc_srv_expected_outputs_cpp,
            default_pb_rpc_cli_expected_outputs_cpp,
        ],
        "tags": ["all", "cpp", "plugins", "mqtt", "pb", "rpc"],
    },
    # ---------------------------------------------mqtt_ros2_rpc---------------------------------
    {
        "script_path": [
            "./start_examples_plugins_mqtt_plugin_ros2_rpc_server.sh",
            "./start_examples_plugins_mqtt_plugin_ros2_rpc_client.sh",
        ],
        "expected_outputs": [
            default_ros2_rpc_srv_expected_outputs_cpp,
            default_ros2_rpc_cli_expected_outputs_cpp,
        ],
        "tags": ["all", "cpp", "plugins", "mqtt", "ros2", "rpc"],
    },
    # ------------------------------------------ros2_pb_chn--------------------------------------
    {
        "script_path": [
            "./start_examples_plugins_ros2_plugin_pb_chn_sub.sh",
            "./start_examples_plugins_ros2_plugin_pb_chn_pub.sh",
        ],
        "expected_outputs": [
            default_pb_chn_sub_expected_outputs_cpp,
            default_pb_chn_pub_expected_outputs_cpp,
        ],
        "tags": ["all", "cpp", "plugins", "ros2_plugin", "pb", "chn"],
    },
    # -------------------------------------------ros2_ros2_chn------------------------------------
    {
        "script_path": [
            "./start_examples_plugins_ros2_plugin_ros2_chn_sub.sh",
            "./start_examples_plugins_ros2_plugin_ros2_chn_pub.sh",
        ],
        "expected_outputs": [
            default_ros2_chn_sub_expected_outputs_cpp,
            default_ros2_chn_pub_expected_outputs_cpp,
        ],
        "tags": ["all", "cpp", "plugins", "ros2_plugin", "ros2", "chn"],
    },
    # --------------------------------------------ros2_pb_rpc-------------------------------------
    {
        "script_path": [
            "./start_examples_plugins_ros2_plugin_pb_rpc_server.sh",
            "./start_examples_plugins_ros2_plugin_pb_rpc_client.sh",
        ],
        "expected_outputs": [
            default_pb_rpc_srv_expected_outputs_cpp,
            default_pb_rpc_cli_expected_outputs_cpp,
        ],
        "tags": ["all", "cpp", "plugins", "ros2_plugin", "pb", "rpc"],
    },
    # ---------------------------------------------ros2_ros2_rpc------------------------------------
    {
        "script_path": [
            "./start_examples_plugins_ros2_plugin_ros2_rpc_server.sh",
            "./start_examples_plugins_ros2_plugin_ros2_rpc_client.sh",
        ],
        "expected_outputs": [
            default_ros2_rpc_srv_expected_outputs_cpp,
            default_ros2_rpc_cli_expected_outputs_cpp,
        ],
        "tags": ["all", "cpp", "plugins", "ros2_plugin", "ros2", "rpc"],
    },
    # --------------------------------------------grpc_pb_rpc----------------------------------------
    {
        "script_path": [
            "./start_examples_plugins_grpc_plugin_pb_rpc_server.sh",
            "./start_examples_plugins_grpc_plugin_pb_rpc_client.sh",
        ],
        "expected_outputs": [
            default_pb_rpc_srv_expected_outputs_cpp,
            default_pb_rpc_cli_expected_outputs_cpp,
        ],
        "tags": ["all", "cpp", "plugins", "grpc", "pb", "rpc"],
        "limit": "port:50050",
    },
    # -------------------------------------------parameter_plugin----------------------------------------
    {
        "script_path": [
            "./start_examples_plugins_parameter_plugin.sh",
            "./tools/parameter_plugin_get_parameter.sh",
        ],
        "expected_outputs": [
            ["Set parameter, key: 'key-1', val: 'val-1'"],
            ['{"code":0,"msg":"","parameter_value":"val-1"}'],
        ],
        "exit_string": [default_exit_string, ""],
        "tags": ["all", "cpp", "plugins", "parameter", "curl"],
        "limit": "port:50080",
    },
    # ------------------------------native(sub)_ros2(pub)_pb_chn----------------------------------
    {
        "script_path": [
            "./native_ros2_pb_chn_subscriber",
            "./start_examples_plugins_ros2_plugin_pb_chn_pub.sh",
        ],
        "expected_outputs": [
            ["msg: count: 1"],
            default_pb_chn_pub_expected_outputs_cpp,
        ],
        "exit_string": [
            "",
            default_exit_string,
        ],
        "tags": ["all", "cpp", "plugins", "ro2_plugin", "pb", "chn", "native"],
    },
    # ------------------------------native(pub)_ros2(sub)_ros2_chn----------------------------------
    {
        "script_path": [
            "./start_examples_plugins_ros2_plugin_ros2_chn_sub.sh",
            "./native_ros2_chn_publisher",
        ],
        "expected_outputs": [
            default_ros2_chn_sub_expected_outputs_cpp,
            ["Publishing msg:"],
        ],
        "exit_string": [
            default_exit_string,
            "",
        ],
        "tags": ["all", "cpp", "plugins", "ro2_plugin", "ros2", "chn", "native"],
    },
    # ------------------------------cpp_ececuotr-----------------------------------------------
    {
        "script_path": [
            "./start_examples_cpp_executor.sh",
        ],
        "expected_outputs": [
            ["Loop count : 1"],
        ],
        "tags": ["all", "cpp", "executor"],
    },
    # ------------------------------cpp_ececuotr_co-----------------------------------------------
    {
        "script_path": [
            "./start_examples_cpp_executor_co.sh",
        ],
        "expected_outputs": [
            ["Loop count : 1"],
        ],
        "tags": ["all", "cpp", "executor"],
    },
    # ------------------------------cpp_ececuotr_co_loop-----------------------------------------------
    {
        "script_path": [
            "./start_examples_cpp_executor_co_loop.sh",
        ],
        "expected_outputs": [
            ["Loop count : 1"],
        ],
        "tags": ["all", "cpp", "executor"],
    },
    # ------------------------------cpp_logger-----------------------------------------------
    {
        "script_path": [
            "./start_examples_cpp_logger.sh",
        ],
        "expected_outputs": [
            ["s = abc, n = 2"],
        ],
        "forbidden_outputs": [
            "",
        ],
        "tags": ["all", "cpp", "logger"],
    },
    # ------------------------------cpp_logger_specify_executor-----------------------------------------------
    {
        "script_path": [
            "./start_examples_cpp_logger_specify_executor.sh",
        ],
        "expected_outputs": [
            ["s = abc, n = 2"],
        ],
        "forbidden_outputs": [
            "",
        ],
        "tags": ["all", "cpp", "logger"],
    },
    # ------------------------------cpp_logger_rotate_file-----------------------------------------------
    {
        "script_path": [
            "./start_examples_cpp_logger_rotate_file.sh",
        ],
        "expected_outputs": [
            ["s = abc, n = 2"],
        ],
        "forbidden_outputs": [
            "",
        ],
        "tags": ["all", "cpp", "logger"],
    },
    # ------------------------------cpp_parameter-----------------------------------------------
    {
        "script_path": [
            "./start_examples_cpp_parameter.sh",
        ],
        "expected_outputs": [
            ["Set parameter, key: 'key-1', val: 'val-1'"],
        ],
        "tags": ["all", "cpp", "parameter"],
    },
    # ------------------------------cpp_pb_chn-----------------------------------------------
    {
        "script_path": [
            "./start_examples_cpp_pb_chn.sh",
        ],
        "expected_outputs": [
            default_pb_chn_pub_expected_outputs_cpp + default_pb_chn_sub_expected_outputs_cpp,
        ],
        "tags": ["all", "cpp", "pb", "chn"],
    },
    # ------------------------------cpp_pb_chn_pub_app-----------------------------------------------
    {
        "script_path": [
            "./start_examples_cpp_pb_chn_publisher_app.sh",
        ],
        "expected_outputs": [
            default_pb_chn_pub_expected_outputs_cpp + default_pb_chn_sub_expected_outputs_cpp,
        ],
        "tags": ["all", "cpp", "pb", "chn", "app"],
    },
    # ------------------------------cpp_pb_chn_sub_app-----------------------------------------------
    {
        "script_path": [
            "./start_examples_cpp_pb_chn_subscriber_app.sh",
        ],
        "expected_outputs": [
            default_pb_chn_pub_expected_outputs_cpp + default_pb_chn_sub_expected_outputs_cpp,
        ],
        "tags": ["all", "cpp", "pb", "chn", "app"],
    },
    # ------------------------------cpp_pb_chn_single_pkg-----------------------------------------------
    {
        "script_path": [
            "./start_examples_cpp_pb_chn_single_pkg.sh",
        ],
        "expected_outputs": [
            default_pb_chn_pub_expected_outputs_cpp + default_pb_chn_sub_expected_outputs_cpp,
        ],
        "tags": ["all", "cpp", "pb", "chn"],
    },
    # ------------------------------cpp_pb_rpc_async-----------------------------------------------
    {
        "script_path": [
            "./start_examples_cpp_pb_rpc_async.sh",
        ],
        "expected_outputs": [
            [
                'Client start new rpc call. req: {"msg":"hello world foo, count 1"}',
                'return rsp: {"code":"0","msg":"echo hello world foo, count 1"}',
                'Client get rpc ret, status: suc, code 0, msg: OK, rsp: {"code":"0","msg":"echo hello world foo, count 1"}',
            ],
        ],
        "tags": ["all", "cpp", "pb", "rpc"],
    },
    # ------------------------------cpp_pb_rpc_co-----------------------------------------------
    {
        "script_path": [
            "./start_examples_cpp_pb_rpc_co.sh",
        ],
        "expected_outputs": [
            [
                "Client start new rpc call.",
                'return rsp: {"code":"0","msg":"echo hello world foo, count 1"}',
                "Client get rpc ret, status: suc, code 0, msg: OK, rsp:",
            ],
        ],
        "tags": ["all", "cpp", "pb", "rpc"],
    },
    # ------------------------------cpp_pb_rpc_future-----------------------------------------------
    {
        "script_path": [
            "./start_examples_cpp_pb_rpc_future.sh",
        ],
        "expected_outputs": [
            [
                'Client start new rpc call. req: {"msg":"hello world foo, count 1"}',
                'return rsp: {"code":"0","msg":"echo hello world foo, count 1"}',
                'Client get rpc ret, status: suc, code 0, msg: OK, rsp: {"code":"0","msg":"echo hello world foo, count 1"}',
            ],
        ],
        "tags": ["all", "cpp", "pb", "rpc"],
    },
    # ------------------------------cpp_pb_rpc_sync-----------------------------------------------
    {
        "script_path": [
            "./start_examples_cpp_pb_rpc_sync.sh",
        ],
        "expected_outputs": [
            [
                "Client start new rpc call.",
                'return rsp: {"code":"0","msg":"echo hello world foo, count 1"}',
                'Client get rpc ret, status: suc, code 0, msg: OK, rsp: {"code":"0","msg":"echo hello world foo, count 1"}',
            ],
        ],
        "tags": ["all", "cpp", "pb", "rpc"],
    },
    # ------------------------------cpp_helloworld-----------------------------------------------
    {
        "script_path": [
            "./start_examples_cpp_helloworld.sh",
        ],
        "expected_outputs": [
            "will waiting for shutdown",
        ],
        "tags": ["all", "cpp", "helloworld"],
    },
    # ------------------------------cpp_helloworld_app-----------------------------------------------
    {
        "script_path": [
            "./start_examples_cpp_helloworld_app_mode.sh",
        ],
        "expected_outputs": [
            "Start succeeded",
        ],
        "tags": ["all", "cpp", "helloworld", "app"],
    },
    # ------------------------------cpp_helloworld_app_registration------------------------------------
    {
        "script_path": [
            "./start_examples_cpp_helloworld_app_registration_mode.sh",
        ],
        "expected_outputs": [
            "will waiting for shutdown",
        ],
        "tags": ["all", "cpp", "helloworld", "app"],
    },
    # ------------------------------cpp_record_plugin_imd------------------------------------
    {
        "script_path": [
            "./start_examples_plugins_record_playback_plugin_record_imd.sh",
        ],
        "expected_outputs": [
            default_pb_chn_pub_expected_outputs_cpp + default_pb_chn_sub_expected_outputs_cpp,
        ],
        "tags": ["all", "cpp", "plugins", "record_playback"],
    },
    # ////////////////////////////////////////////////////////////////////////////////////////////////////
    # ////////////////////////////////////PYTHON EXAMPLES/////////////////////////////////////////////////
    # ///////////////////////////////////////////////////////////////////////////////////////////////////
    # ---------------------------------python_helloworld--------------------------------------------
    {
        "script_path": [
            "./start_examples_py_helloworld_app_mode.sh",
        ],
        "expected_outputs": [
            ["{'key1': 'val1', 'key2': 'val2'}", "Loop count: 1", "Loop count: 2"],
        ],
        "tags": ["all", "python", "helloworld", "app"],
        "cwd": py_cwd + "/helloworld",
    },
    # ---------------------------------python_helloworld_registration--------------------------------
    {
        "script_path": [
            "./start_examples_py_helloworld_registration_mode.sh",
        ],
        "expected_outputs": [
            ["{'key1': 'val1', 'key2': 'val2'}"] + ["run test task"],
        ],
        "tags": ["all", "python", "helloworld", "app"],
        "cwd": py_cwd + "/helloworld",
    },
    # ---------------------------------python_http_pb_rpc----------------------------------------
    {
        "script_path": [
            "./start_examples_py_pb_rpc_http_server.sh",
            "./start_examples_py_pb_rpc_http_client.sh",
        ],
        "expected_outputs": [
            default_pb_rpc_srv_expected_outputs_python,
            default_pb_rpc_cli_expected_outputs_python,
        ],
        "tags": ["all", "python", "plugins", "net_plugin", "http", "pb", "rpc"],
        "cwd": py_cwd + "/pb_rpc",
        "limit": "port:50080",
    },
    # ---------------------------------python_grpc_pb_rpc----------------------------------------
    {
        "script_path": [
            "./start_examples_py_pb_rpc_grpc_server.sh",
            "./start_examples_py_pb_rpc_grpc_client.sh",
        ],
        "expected_outputs": [
            default_pb_rpc_srv_expected_outputs_python,
            default_pb_rpc_cli_expected_outputs_python,
        ],
        "tags": ["all", "python", "plugins", "grpc_plugin", "pb", "rpc"],
        "cwd": py_cwd + "/pb_rpc",
        "limit": "port:50050",
    },
    # ---------------------------------python_ros_plugin_pb_rpc----------------------------------------
    {
        "script_path": [
            "./start_examples_py_pb_rpc_ros2_server.sh",
            "./start_examples_py_pb_rpc_ros2_client.sh",
        ],
        "expected_outputs": [
            default_pb_rpc_srv_expected_outputs_python,
            default_pb_rpc_cli_expected_outputs_python,
        ],
        "tags": ["all", "python", "plugins", "ros2_plugin", "pb", "rpc"],
        "cwd": py_cwd + "/pb_rpc",
    },
    # ---------------------------------python_http_pb_chn----------------------------------------
    {
        "script_path": [
            "./start_examples_py_pb_chn_http_sub.sh",
            "./start_examples_py_pb_chn_http_pub.sh",
        ],
        "expected_outputs": [
            default_pb_chn_sub_expected_outputs_python,
            default_pb_chn_pub_expected_outputs_python,
        ],
        "tags": ["all", "python", "plugins", "net_plugin", "http", "pb", "chn"],
        "cwd": py_cwd + "/pb_chn",
        "limit": "port:50080",
    },
    # ---------------------------------python_ros2_plugin_pb_chn----------------------------------------
    {
        "script_path": [
            "./start_examples_py_pb_chn_ros2_sub.sh",
            "./start_examples_py_pb_chn_ros2_pub.sh",
        ],
        "expected_outputs": [
            default_pb_chn_sub_expected_outputs_python,
            default_pb_chn_pub_expected_outputs_python,
        ],
        "tags": ["all", "python", "plugins", "ros2_plugin", "pb", "chn"],
        "cwd": py_cwd + "/pb_chn",
    },
    # ---------------------------------python_parameter----------------------------------------
    {
        "script_path": [
            "./start_examples_py_parameter.sh",
        ],
        "expected_outputs": [
            [
                "Start SetParameterLoop.",
                "SetParameterLoop count: 1 -------------------------",
                "Set parameter, key: 'key-1', val: 'val-1'",
                "Start GetParameterLoop.",
                "GetParameterLoop count: 1 -------------------------",
                "Get parameter, key: 'key-1', val: 'val-1'",
            ],
        ],
        "tags": ["all", "python", "parameter"],
        "cwd": py_cwd + "/parameter",
    },
    # ---------------------------------python_http_ros2_chn----------------------------------------
    {
        "script_path": [
            "./start_examples_py_ros2_chn_http_sub.sh",
            "./start_examples_py_ros2_chn_http_pub.sh",
        ],
        "expected_outputs": [
            default_ros2_chn_sub_expected_outputs_python,
            default_ros2_chn_pub_expected_outputs_python,
        ],
        "tags": ["all", "python", "plugins", "net_plugin", "http", "ros2", "chn"],
        "cwd": py_cwd + "/ros2_chn",
        "limit": "port:50080",
    },
    # ---------------------------------python_ros2_ros2_chn----------------------------------------
    {
        "script_path": [
            "./start_examples_py_ros2_chn_ros2_sub.sh",
            "./start_examples_py_ros2_chn_ros2_pub.sh",
        ],
        "expected_outputs": [
            default_ros2_chn_sub_expected_outputs_python,
            default_ros2_chn_pub_expected_outputs_python,
        ],
        "tags": ["all", "python", "plugins", "ros2_plugin", "ros2", "chn"],
        "cwd": py_cwd + "/ros2_chn",
    },
    # ---------------------------------python_http_ros2_rpc----------------------------------------
    {
        "script_path": [
            "./start_examples_py_ros2_rpc_http_server.sh",
            "./start_examples_py_ros2_rpc_http_client.sh",
        ],
        "expected_outputs": [
            default_ros2_rpc_srv_expected_outputs_python,
            default_ros2_rpc_cli_expected_outputs_python,
        ],
        "tags": ["all", "python", "plugins", "net_plugin", "http", "ros2", "rpc"],
        "cwd": py_cwd + "/ros2_rpc",
        "limit": "port:50080",
    },
    # ---------------------------------python_grpc_ros2_rpc----------------------------------------
    {
        "script_path": [
            "./start_examples_py_ros2_rpc_grpc_server.sh",
            "./start_examples_py_ros2_rpc_grpc_client.sh",
        ],
        "expected_outputs": [
            default_ros2_rpc_srv_expected_outputs_python,
            default_ros2_rpc_cli_expected_outputs_python,
        ],
        "tags": ["all", "python", "plugins", "grpc_plugin", "ros2", "rpc"],
        "cwd": py_cwd + "/ros2_rpc",
        "limit": "port:50050",
    },
    # ---------------------------------python_ros2_ros2_rpc----------------------------------------
    {
        "script_path": [
            "./start_examples_py_ros2_rpc_ros2_server.sh",
            "./start_examples_py_ros2_rpc_ros2_client.sh",
        ],
        "expected_outputs": [
            default_ros2_rpc_srv_expected_outputs_python,
            default_ros2_rpc_cli_expected_outputs_python,
        ],
        "tags": ["all", "python", "plugins", "ros2_plugin", "ros2", "rpc"],
        "cwd": py_cwd + "/ros2_rpc",
    },
]
