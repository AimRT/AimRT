# Copyright (c) 2024 The AimRT Authors.
# AimRT is licensed under Mulan PSL v2.

import argparse
import signal
import sys
import threading

import aimrt_py
import benchmark_subscriber_module

global_aimrt_core = None


def signal_handler(sig, _):
    global global_aimrt_core

    if (global_aimrt_core and (sig == signal.SIGINT or sig == signal.SIGTERM)):
        global_aimrt_core.Shutdown()
        return

    sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description='Example benchmark subscriber application.')
    parser.add_argument('--cfg_file_path', type=str, default="", help='config file path')
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("AimRT start.")

    aimrt_core = aimrt_py.Core()

    global global_aimrt_core
    global_aimrt_core = aimrt_core

    module = benchmark_subscriber_module.BenchmarkSubscriber()
    aimrt_core.RegisterModule(module)

    core_options = aimrt_py.CoreOptions()
    core_options.cfg_file_path = args.cfg_file_path
    aimrt_core.Initialize(core_options)

    thread = threading.Thread(target=aimrt_core.Start)
    thread.start()

    while thread.is_alive():
        thread.join(1.0)

    aimrt_core.Shutdown()

    global_aimrt_core = None

    print("AimRT exit.")


if __name__ == '__main__':
    main()
