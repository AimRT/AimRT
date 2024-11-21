# Copyright (c) 2024 The AimRT Authors.
# AimRT is licensed under Mulan PSL v2.

import argparse
import threading
import time

import aimrt_py
import yaml
from example_ros2.msg import RosTestMsg


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
    assert publisher, f"Get publisher for topic '{topic_name}' failed."

    aimrt_py.RegisterPublishType(publisher, RosTestMsg)

    # Start
    thread = threading.Thread(target=aimrt_core.Start)
    thread.start()

    # Sleep for seconds
    time.sleep(1)

    msg = RosTestMsg()
    msg.num = 1000

    # Publish event
    msg.data = [bytes([x]) for x in [1, 2, 3, 4]]
    aimrt_py.Publish(publisher, msg)
    aimrt_py.info(module_handle.GetLogger(),
                  f"Publish new ros2 message, data: {msg.data}")

    # Publish event with ros2 serialization
    msg.data = [bytes([x]) for x in [5, 6, 7, 8]]
    aimrt_py.Publish(publisher, "ros2", msg)
    aimrt_py.info(module_handle.GetLogger(),
                  f"Publish new ros2 message, data: {msg.data}")

    # Publish event with context
    ctx = aimrt_py.Context()
    ctx.SetMetaValue("key1", "value1")
    msg.data = [bytes([x]) for x in [9, 10, 11, 12]]
    aimrt_py.Publish(publisher, ctx, msg)
    aimrt_py.info(module_handle.GetLogger(),
                  f"Publish new ros2 message, data: {msg.data}")

    # Publish event with context ref
    ctx.Reset()  # Reset context, then it can be used again
    ctx_ref = aimrt_py.ContextRef(ctx)
    ctx_ref.SetMetaValue("key2", "value2")
    ctx_ref.SetSerializationType("ros2")
    msg.data = [bytes([x]) for x in [13, 14, 15, 16]]
    aimrt_py.Publish(publisher, ctx_ref, msg)
    aimrt_py.info(module_handle.GetLogger(),
                  f"Publish new ros2 message, data: {msg.data}")

    # Sleep for seconds
    time.sleep(1)

    # Shutdown
    aimrt_core.Shutdown()

    thread.join()

    print("AimRT exit.")


if __name__ == '__main__':
    main()
