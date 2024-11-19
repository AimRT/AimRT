# Copyright (c) 2024 The AimRT Authors.
# AimRT is licensed under Mulan PSL v2.

import argparse
import signal
import sys
import threading

import aimrt_py
import example_ros2.msg
import example_ros2.srv
import RosTestRpc_aimrt_rpc_ros2

global_aimrt_core = None


def signal_handler(sig, frame):
    global global_aimrt_core

    if (global_aimrt_core and (sig == signal.SIGINT or sig == signal.SIGTERM)):
        global_aimrt_core.Shutdown()
        return

    sys.exit(0)


class RosTestRpcImpl(RosTestRpc_aimrt_rpc_ros2.RosTestRpcService):
    def __init__(self, logger):
        super().__init__()
        self.logger = logger

    @staticmethod
    def PrintMetaInfo(logger, ctx_ref):
        meta_keys = ctx_ref.GetMetaKeys()
        for key in meta_keys:
            aimrt_py.info(logger, f"meta key: {key}, value: {ctx_ref.GetMetaValue(key)}")

    def RosTestRpc(self, ctx_ref, req):
        rsp = example_ros2.srv.RosTestRpc.Response()
        rsp.code = 1000
        rsp.string_data = "Hello, AimRT!"
        rsp.wstring_data = "Hello, AimRT!1111111111111111"
        rsp.bool_data = True
        rsp.byte_data = bytes([1])
        rsp.char_data = 92
        rsp.float32_data = float(1.1)
        rsp.float64_data = float(2.2)
        rsp.int8_data = int(-8)
        rsp.uint8_data = int(8)
        rsp.int16_data = int(-16)
        rsp.uint16_data = int(16)
        rsp.int32_data = int(-32)
        rsp.uint32_data = int(32)
        rsp.int64_data = int(-64)
        rsp.uint64_data = int(64)

        rsp.bool_static_array_data = [True, False, True]
        rsp.byte_static_array_data = [bytes([1]), bytes([2]), bytes([3])]
        rsp.char_static_array_data = [93, 94, 95]
        rsp.float32_static_array_data = [float(1.1), float(2.2), float(3.3)]
        rsp.float64_static_array_data = [float(1.1), float(2.2), float(3.3)]
        rsp.int8_static_array_data = [int(-8), int(-7), int(-6)]
        rsp.uint8_static_array_data = [int(8), int(7), int(6)]
        rsp.int16_static_array_data = [int(-16), int(-15), int(-14)]
        rsp.uint16_static_array_data = [int(16), int(15), int(14)]
        rsp.int32_static_array_data = [int(-32), int(-31), int(-30)]
        rsp.uint32_static_array_data = [int(32), int(31), int(30)]
        rsp.int64_static_array_data = [int(-64), int(-63), int(-62)]
        rsp.uint64_static_array_data = [int(64), int(63), int(62)]
        rsp.string_static_array_data = ["Hello", "AimRT", "World"]
        rsp.wstring_static_array_data = ["Hello", "AimRT", "World"]

        rsp.bool_dynamic_array_data = [True, False, True]
        rsp.byte_dynamic_array_data = [bytes([1]), bytes([2]), bytes([3])]
        rsp.char_dynamic_array_data = [93, 94, 95]
        rsp.float32_dynamic_array_data = [float(1.1), float(2.2), float(3.3)]
        rsp.float64_dynamic_array_data = [float(1.1), float(2.2), float(3.3)]
        rsp.int8_dynamic_array_data = [int(-8), int(-7), int(-6)]
        rsp.uint8_dynamic_array_data = [int(8), int(7), int(6)]
        rsp.int16_dynamic_array_data = [int(-16), int(-15), int(-14)]
        rsp.uint16_dynamic_array_data = [int(16), int(15), int(14)]
        rsp.int32_dynamic_array_data = [int(-32), int(-31), int(-30)]
        rsp.uint32_dynamic_array_data = [int(32), int(31), int(30)]
        rsp.int64_dynamic_array_data = [int(-64), int(-63), int(-62)]
        rsp.uint64_dynamic_array_data = [int(64), int(63), int(62)]
        rsp.string_dynamic_array_data = ["Hello", "AimRT", "World"]
        rsp.wstring_dynamic_array_data = ["Hello", "AimRT", "World"]

        rsp.ros_test_data.num = 100
        rsp.ros_test_data.num2 = float(1.1)
        rsp.ros_test_data.data = 10

        rsp.ros_test_data_static_array = [example_ros2.msg.RosTestData(num=200, num2=float(2.2), data=20),
                                          example_ros2.msg.RosTestData(num=300, num2=float(3.3), data=30),
                                          example_ros2.msg.RosTestData(num=400, num2=float(4.4), data=40)]

        rsp.ros_test_data_dynamic_array = [example_ros2.msg.RosTestData(num=500, num2=float(5.5), data=50),
                                           example_ros2.msg.RosTestData(num=600, num2=float(6.6), data=60),
                                           example_ros2.msg.RosTestData(num=700, num2=float(7.7), data=70)]

        RosTestRpcImpl.PrintMetaInfo(self.logger, ctx_ref)
        aimrt_py.info(self.logger,
                      f"Server handle new rpc call. "
                      f"context: {ctx_ref.ToString()}, "
                      f"req: {req}, "
                      f"return rsp: {rsp}")

        return aimrt_py.RpcStatus(), rsp


def main():

    parser = argparse.ArgumentParser(description='Example rpc server app.')
    parser.add_argument('--cfg_file_path', type=str, default="", help='config file path')
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("AimRT start.")

    aimrt_core = aimrt_py.Core()

    global global_aimrt_core
    global_aimrt_core = aimrt_core

    # Initialize
    core_options = aimrt_py.CoreOptions()
    core_options.cfg_file_path = args.cfg_file_path
    aimrt_core.Initialize(core_options)

    # Create Module
    module_handle = aimrt_core.CreateModule("NormalRpcServerPyModule")

    # Register rpc service
    service = RosTestRpcImpl(module_handle.GetLogger())
    ret = module_handle.GetRpcHandle().RegisterService(service)
    assert ret, "Register service failed."

    # Start
    thread = threading.Thread(target=aimrt_core.Start)
    thread.start()

    while thread.is_alive():
        thread.join(1.0)

    print("AimRT exit.")


if __name__ == '__main__':
    main()
