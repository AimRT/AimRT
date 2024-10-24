# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import argparse
import threading
import time

import aimrt_py
import event_pb2
import yaml
from google.protobuf.json_format import MessageToJson


def main():
    parser = argparse.ArgumentParser(description='Example channel publisher app.')
    parser.add_argument('--cfg_file_path', type=str, default="", help='config file path')
    args = parser.parse_args()

    print("AimRT start.")

    aimrt_core = aimrt_py.Core()

    # Initialize
    core_options = aimrt_py.CoreOptions()
    core_options.cfg_file_path = args.cfg_file_path
    aimrt_core.Initialize(core_options)

    # Create Module
    module_handle = aimrt_core.CreateModule("NormalPublisherPyModule")

    # Read cfg
    module_cfg_file_path = module_handle.GetConfigurator().GetConfigFilePath()
    with open(module_cfg_file_path, 'r') as file:
        data = yaml.safe_load(file)
        topic_name = str(data["topic_name"])

    # Register publish type
    publisher = module_handle.GetChannelHandle().GetPublisher(topic_name)
    assert publisher, "Get publisher for topic '{}' failed.".format(topic_name)

    aimrt_py.RegisterPublishType(publisher, event_pb2.ExampleEventMsg)

    # Start
    thread = threading.Thread(target=aimrt_core.Start)
    thread.start()

    # Sleep for seconds
    time.sleep(1)

    # Publish event
    event_msg = event_pb2.ExampleEventMsg()
    event_msg.msg = "example msg"
    event_msg.num = 123456

    aimrt_py.info(module_handle.GetLogger(),
                  "Publish new pb event, data: {}".format(MessageToJson(event_msg)))

    aimrt_py.Publish(publisher, event_msg)

    # Sleep for seconds
    time.sleep(1)

    # Shutdown
    aimrt_core.Shutdown()

    thread.join()

    print("AimRT exit.")


if __name__ == '__main__':
    main()
