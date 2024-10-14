# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import argparse
import signal
import sys
import threading
import time

import aimrt_py

global_aimrt_core = None
running_flag = True


def signal_handler(sig, _):
    global global_aimrt_core
    global running_flag

    if (global_aimrt_core and (sig == signal.SIGINT or sig == signal.SIGTERM)):
        global_aimrt_core.Shutdown()
        running_flag = False
        return

    sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description='Example parameter app.')
    parser.add_argument('--cfg_file_path', type=str, default="", help='config file path')
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("AimRT start.")

    aimrt_core = aimrt_py.Core()

    global global_aimrt_core
    global_aimrt_core = aimrt_core

    core_options = aimrt_py.CoreOptions()
    core_options.cfg_file_path = args.cfg_file_path
    aimrt_core.Initialize(core_options)

    module_handle = aimrt_core.CreateModule("ParameterModule")
    assert (isinstance(module_handle, aimrt_py.CoreRef))

    parameter_handle = module_handle.GetParameterHandle()

    # Start aimrt core
    thread_start = threading.Thread(target=aimrt_core.Start)
    thread_start.start()

    time.sleep(1)

    def SetParameterLoop():
        aimrt_py.info(module_handle.GetLogger(), "Start SetParameterLoop.")
        count = 0
        global running_flag
        while running_flag:
            count += 1
            aimrt_py.info(module_handle.GetLogger(), f"SetParameterLoop count: {count} -------------------------")
            parameter_handle.SetParameter(f"key-{count}", f"val-{count}")
            aimrt_py.info(module_handle.GetLogger(), f"Set parameter, key: 'key-{count}', val: 'val-{count}'")
            time.sleep(1)
        aimrt_py.info(module_handle.GetLogger(), "SetParameterLoop stopped.")

    def GetParameterLoop():
        aimrt_py.info(module_handle.GetLogger(), "Start GetParameterLoop.")
        count = 0
        global running_flag
        while running_flag:
            count += 1
            aimrt_py.info(module_handle.GetLogger(), f"GetParameterLoop count: {count} -------------------------")
            value = parameter_handle.GetParameter(f"key-{count}")
            aimrt_py.info(module_handle.GetLogger(), f"Get parameter, key: 'key-{count}', val: '{value}'")
            time.sleep(1)
        aimrt_py.info(module_handle.GetLogger(), "GetParameterLoop stopped.")

    thread_set_parameter = threading.Thread(target=SetParameterLoop)
    thread_get_parameter = threading.Thread(target=GetParameterLoop)

    thread_set_parameter.start()
    time.sleep(0.5)
    thread_get_parameter.start()

    while thread_start.is_alive():
        thread_start.join(1.0)

    while thread_set_parameter.is_alive():
        thread_set_parameter.join(1.0)

    while thread_get_parameter.is_alive():
        thread_get_parameter.join(1.0)

    global_aimrt_core = None

    print("AimRT exit.")


if __name__ == "__main__":
    main()
