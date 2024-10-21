# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import google.protobuf
import google.protobuf.message

from . import aimrt_python_runtime


def RegisterPublishType(publisher, protobuf_type):
    aimrt_ts = aimrt_python_runtime.TypeSupport()
    aimrt_ts.SetTypeName("pb:" + protobuf_type.DESCRIPTOR.full_name)
    aimrt_ts.SetSerializationTypesSupportedList(["pb", "json"])

    return publisher.RegisterPublishType(aimrt_ts)


def Publish(publisher, pb_msg):
    publisher.Publish("pb:" + pb_msg.DESCRIPTOR.full_name, "pb", pb_msg.SerializeToString())

def PublishWithCtx(publisher: aimrt_python_runtime.PublisherRef,
                   ctx: aimrt_python_runtime.ContextRef | aimrt_python_runtime.Context,
                   pb_msg: google.protobuf.message.Message):
    if isinstance(ctx, aimrt_python_runtime.Context):
        ctx_ref = aimrt_python_runtime.ContextRef(ctx)
    elif isinstance(ctx, aimrt_python_runtime.ContextRef):
        ctx_ref = ctx
    else:
        raise TypeError("ctx must be 'aimrt_python_runtime.Context' or 'aimrt_python_runtime.ContextRef'")

    publisher.PublishWithCtx("pb:" + pb_msg.DESCRIPTOR.full_name, ctx_ref, pb_msg.SerializeToString())

def Subscribe(subscriber, protobuf_type, callback):
    aimrt_ts = aimrt_python_runtime.TypeSupport()
    aimrt_ts.SetTypeName("pb:" + protobuf_type.DESCRIPTOR.full_name)
    aimrt_ts.SetSerializationTypesSupportedList(["pb", "json"])

    def handle_callback(serialization_type, msg_buf):
        try:
            if serialization_type == "pb":
                msg = protobuf_type()
                msg.ParseFromString(msg_buf)
                callback(msg)
                return

            if serialization_type == "json":
                msg = protobuf_type()
                google.protobuf.json_format.Parse(msg_buf, msg)
                callback(msg)
                return
        except Exception as e:
            print("AimRT channel handle get exception, {}".format(e))
            return

    subscriber.Subscribe(aimrt_ts, handle_callback)
