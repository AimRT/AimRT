# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

from typing import Callable

import google.protobuf
import google.protobuf.message

from . import aimrt_python_runtime


def RegisterPublishType(publisher: aimrt_python_runtime.PublisherRef, protobuf_type: google.protobuf.message.Message):
    aimrt_ts = aimrt_python_runtime.TypeSupport()
    aimrt_ts.SetTypeName("pb:" + protobuf_type.DESCRIPTOR.full_name)
    aimrt_ts.SetSerializationTypesSupportedList(["pb", "json"])

    return publisher.RegisterPublishType(aimrt_ts)


def PublishPb(publisher: aimrt_python_runtime.PublisherRef, pb_msg: google.protobuf.message.Message):
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

    serialization_type = ctx_ref.GetSerializationType()
    if serialization_type == "pb":
        serialized_msg = pb_msg.SerializeToString()
    elif serialization_type == "json":
        serialized_msg = google.protobuf.json_format.MessageToJson(pb_msg)
    else:
        raise ValueError(f"Invalid serialization type: {ctx_ref.GetSerializationType()}")

    publisher.PublishWithCtx(f"pb:{pb_msg.DESCRIPTOR.full_name}", ctx_ref, serialized_msg)


def Publish(publisher: aimrt_python_runtime.PublisherRef, second, third=None):
    if isinstance(second, google.protobuf.message.Message):
        pb_msg = second
        ctx = third
    elif isinstance(second, (aimrt_python_runtime.Context, aimrt_python_runtime.ContextRef)) \
            and isinstance(third, google.protobuf.message.Message):
        ctx = second
        pb_msg = third
    else:
        raise TypeError(f"Publish invalid arguments, second: {type(second)}, third: {type(third)}"
                        "Must be 'google.protobuf.message.Message' and "
                        "'aimrt_python_runtime.Context' or 'aimrt_python_runtime.ContextRef'")

    if ctx is None:
        PublishPb(publisher, pb_msg)
    else:
        PublishWithCtx(publisher, ctx, pb_msg)


def Subscribe(subscriber: aimrt_python_runtime.SubscriberRef,
              protobuf_type: google.protobuf.message.Message,
              callback: Callable[[google.protobuf.message.Message], None]):
    aimrt_ts = aimrt_python_runtime.TypeSupport()
    aimrt_ts.SetTypeName("pb:" + protobuf_type.DESCRIPTOR.full_name)
    aimrt_ts.SetSerializationTypesSupportedList(["pb", "json"])

    def handle_callback(serialization_type: str, msg_buf: bytes):
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


def SubscribeWithCtx(subscriber: aimrt_python_runtime.SubscriberRef,
                     protobuf_type: google.protobuf.message.Message,
                     callback: Callable[[aimrt_python_runtime.ContextRef, google.protobuf.message.Message], None]):
    aimrt_ts = aimrt_python_runtime.TypeSupport()
    aimrt_ts.SetTypeName("pb:" + protobuf_type.DESCRIPTOR.full_name)
    aimrt_ts.SetSerializationTypesSupportedList(["pb", "json"])

    def handle_callback(ctx_ref: aimrt_python_runtime.ContextRef, msg_buf: bytes):
        try:
            if ctx_ref.GetSerializationType() == "pb":
                msg = protobuf_type()
                msg.ParseFromString(msg_buf)
                callback(ctx_ref, msg)
                return

            if ctx_ref.GetSerializationType() == "json":
                msg = protobuf_type()
                google.protobuf.json_format.Parse(msg_buf, msg)
                callback(ctx_ref, msg)
                return
        except Exception as e:
            print("AimRT channel handle get exception, {}".format(e))
            return

    subscriber.SubscribeWithCtx(aimrt_ts, handle_callback)
