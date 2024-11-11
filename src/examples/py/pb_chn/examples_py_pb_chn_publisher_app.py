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

    event_msg = event_pb2.ExampleEventMsg()

    # Publish event
    event_msg.msg = "Publish without ctx or serialization_type"
    event_msg.num = 1
    aimrt_py.Publish(publisher, event_msg)
    aimrt_py.info(module_handle.GetLogger(),
                  f"Publish new pb event, data: {MessageToJson(event_msg)}")

    # Publish event with json serialization
    event_msg.msg = "Publish with json serialization"
    event_msg.num = 2
    aimrt_py.Publish(publisher, "json", event_msg)
    aimrt_py.info(module_handle.GetLogger(),
                  f"Publish new pb event, data: {MessageToJson(event_msg)}")

    # Publish event with context
    ctx = aimrt_py.Context()
    ctx.SetMetaValue("key1", "value1")
    event_msg.msg = "Publish with context"
    event_msg.num = 3
    aimrt_py.Publish(publisher, ctx, event_msg)
    aimrt_py.info(module_handle.GetLogger(),
                  f"Publish new pb event, data: {MessageToJson(event_msg)}")

    # Publish event with context ref
    ctx.Reset()  # Reset context, then it can be used again
    ctx_ref = aimrt_py.ContextRef(ctx)
    ctx_ref.SetMetaValue("key2", "value2")
    ctx_ref.SetSerializationType("json")
    event_msg.msg = "Publish with context ref"
    event_msg.num = 4
    aimrt_py.Publish(publisher, ctx_ref, event_msg)
    aimrt_py.info(module_handle.GetLogger(),
                  f"Publish new pb event, data: {MessageToJson(event_msg)}")

    # Sleep for seconds
    time.sleep(1)

    # Shutdown
    aimrt_core.Shutdown()

    thread.join()

    print("AimRT exit.")


if __name__ == '__main__':
    main()
