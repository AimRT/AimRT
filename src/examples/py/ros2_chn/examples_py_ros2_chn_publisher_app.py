# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import argparse
import threading
import time

import aimrt_py
import yaml
from std_msgs.msg import String


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

    aimrt_py.RegisterPublishType(publisher, String)

    # Start
    thread = threading.Thread(target=aimrt_core.Start)
    thread.start()

    # Sleep for seconds
    time.sleep(1)

    msg = String()

    # Publish event
    msg.data = "Publish without ctx or serialization_type"
    aimrt_py.Publish(publisher, msg)
    aimrt_py.info(module_handle.GetLogger(),
                  f"Publish new ros2 message, data: {msg.data}")

    # Publish event with ros2 serialization
    msg.data = "Publish with ros2 serialization"
    aimrt_py.Publish(publisher, "ros2", msg)
    aimrt_py.info(module_handle.GetLogger(),
                  f"Publish new ros2 message, data: {msg.data}")

    # Publish event with context
    ctx = aimrt_py.Context()
    ctx.SetMetaValue("key1", "value1")
    msg.data = "Publish with context"
    aimrt_py.Publish(publisher, ctx, msg)
    aimrt_py.info(module_handle.GetLogger(),
                  f"Publish new ros2 message, data: {msg.data}")

    # Publish event with context ref
    ctx.Reset()  # Reset context, then it can be used again
    ctx_ref = aimrt_py.ContextRef(ctx)
    ctx_ref.SetMetaValue("key2", "value2")
    ctx_ref.SetSerializationType("ros2")
    msg.data = "Publish with context ref"
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
