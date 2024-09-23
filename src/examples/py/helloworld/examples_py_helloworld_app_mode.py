# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import argparse
import threading
import signal
import sys
import aimrt_py
import yaml
import time

global_aimrt_core = None
running_flag = True


def signal_handler(sig, frame):
    global global_aimrt_core
    global running_flag

    if (global_aimrt_core and (sig == signal.SIGINT or sig == signal.SIGTERM)):
        global_aimrt_core.Shutdown()
        running_flag = False
        return

    sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description='Example helloworld app mode.')
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

    module_handle = aimrt_core.CreateModule("HelloWorldPyModule")
    assert (isinstance(module_handle, aimrt_py.CoreRef))

    aimrt_py.info(module_handle.GetLogger(), "This is an example log.")

    # Read cfg
    module_cfg_file_path = module_handle.GetConfigurator().GetConfigFilePath()
    with open(module_cfg_file_path, 'r') as file:
        data = yaml.safe_load(file)
        aimrt_py.info(module_handle.GetLogger(), str(data))

    # Start aimrt core
    thread_start = threading.Thread(target=aimrt_core.Start)
    thread_start.start()

    time.sleep(1)

    # Start loop
    def Loop():
        count = 1
        global running_flag
        while (running_flag):
            aimrt_py.info(module_handle.GetLogger(), "Loop count: {}".format(count))
            count = count + 1
            time.sleep(1)

    thread_loop = threading.Thread(target=Loop)
    thread_loop.start()

    # wait for shutdown
    while thread_start.is_alive():
        thread_start.join(1.0)

    while thread_loop.is_alive():
        thread_loop.join(1.0)

    global_aimrt_core = None

    print("AimRT exit.")


if __name__ == '__main__':
    main()
