# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import argparse
import signal
import sys
import threading

import aimrt_py
import event_pb2
import yaml
from google.protobuf.json_format import MessageToJson

global_aimrt_core = None


def signal_handler(sig, frame):
    global global_aimrt_core

    if (global_aimrt_core and (sig == signal.SIGINT or sig == signal.SIGTERM)):
        global_aimrt_core.Shutdown()
        return

    sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description='Example channel subscriber app.')
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
    module_handle = aimrt_core.CreateModule("NormalSubscriberPyModule")

    # Read cfg
    module_cfg_file_path = module_handle.GetConfigurator().GetConfigFilePath()
    with open(module_cfg_file_path, 'r') as file:
        data = yaml.safe_load(file)
        topic_name = str(data["topic_name"])

    # Subscribe
    subscriber = module_handle.GetChannelHandle().GetSubscriber(topic_name)
    assert subscriber, f"Get subscriber for topic '{topic_name}' failed."

    def EventHandle(ctx_ref, msg):
        aimrt_py.info(module_handle.GetLogger(),
                      f"Get new pb event, ctx: {ctx_ref.ToString()}, data: {MessageToJson(msg)}")

    aimrt_py.Subscribe(subscriber, event_pb2.ExampleEventMsg, EventHandle)

    # Start
    thread = threading.Thread(target=aimrt_core.Start)
    thread.start()

    while thread.is_alive():
        thread.join(1.0)

    print("AimRT exit.")


if __name__ == '__main__':
    main()
