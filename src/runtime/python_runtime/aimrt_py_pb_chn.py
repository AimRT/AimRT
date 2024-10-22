# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import inspect
from typing import Callable

import google.protobuf
import google.protobuf.message

from . import aimrt_python_runtime


def RegisterPublishType(publisher: aimrt_python_runtime.PublisherRef, protobuf_type: google.protobuf.message.Message):
    aimrt_ts = aimrt_python_runtime.TypeSupport()
    aimrt_ts.SetTypeName("pb:" + protobuf_type.DESCRIPTOR.full_name)
    aimrt_ts.SetSerializationTypesSupportedList(["pb", "json"])

    return publisher.RegisterPublishType(aimrt_ts)


def SerializeProtobufMessage(pb_msg: google.protobuf.message.Message, serialization_type: str) -> bytes:
    if serialization_type == "pb":
        return pb_msg.SerializeToString()
    elif serialization_type == "json":
        return google.protobuf.json_format.MessageToJson(pb_msg)
    else:
        raise ValueError(f"Invalid serialization type: {serialization_type}")


def PublishWithSerializationType(publisher: aimrt_python_runtime.PublisherRef,
                                 serialization_type: str,
                                 pb_msg: google.protobuf.message.Message):
    serialized_msg = SerializeProtobufMessage(pb_msg, serialization_type)
    publisher.PublishWithSerializationType("pb:" + pb_msg.DESCRIPTOR.full_name, serialization_type, serialized_msg)


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
    serialized_msg = SerializeProtobufMessage(pb_msg, serialization_type)

    publisher.PublishWithCtx(f"pb:{pb_msg.DESCRIPTOR.full_name}", ctx_ref, serialized_msg)


def Publish(publisher: aimrt_python_runtime.PublisherRef, second, third=None):
    """Publish a message to a channel.

    This function can be called in following ways:
    - Publish(publisher, pb_msg)
    - Publish(publisher, pb_msg, ctx)
    - Publish(publisher, pb_msg, serialization_type)
    - Publish(publisher, ctx, pb_msg)
    - Publish(publisher, serialization_type, pb_msg)

    pb_msg: google.protobuf.message.Message
    ctx: aimrt_python_runtime.Context or aimrt_python_runtime.ContextRef
    serialization_type: str

    Args:
        publisher (aimrt_python_runtime.PublisherRef): channel publisher
        second: pb_msg or ctx or serialization_type
        third: pb_msg or ctx or serialization_type or None
    Raises:
        TypeError: if the arguments are invalid
    """
    if isinstance(second, google.protobuf.message.Message):
        pb_msg = second
        ctx = third
    elif isinstance(third, google.protobuf.message.Message):
        pb_msg = third
        ctx = second
    else:
        raise TypeError("Invalid arguments, no protobuf message found")

    if isinstance(ctx, (aimrt_python_runtime.Context, aimrt_python_runtime.ContextRef)):
        PublishWithCtx(publisher, ctx, pb_msg)
    elif isinstance(ctx, str):
        PublishWithSerializationType(publisher, ctx, pb_msg)
    elif ctx is None:
        # default use pb serialization
        PublishWithSerializationType(publisher, "pb", pb_msg)
    else:
        raise TypeError(
            f"Invalid context type: {type(ctx)}, "
            f"only 'aimrt_python_runtime.Context' or 'aimrt_python_runtime.ContextRef' or 'str' is supported")


def DeserializeProtobufMessage(msg_buf: bytes,
                               serialization_type: str,
                               protobuf_type: google.protobuf.message.Message) -> google.protobuf.message.Message:
    msg = protobuf_type()
    if serialization_type == "pb":
        msg.ParseFromString(msg_buf)
    elif serialization_type == "json":
        google.protobuf.json_format.Parse(msg_buf, msg)
    else:
        raise ValueError(f"Invalid serialization type: {serialization_type}")
    return msg


def Subscribe(subscriber: aimrt_python_runtime.SubscriberRef,
              protobuf_type: google.protobuf.message.Message,
              callback: Callable):
    aimrt_ts = aimrt_python_runtime.TypeSupport()
    aimrt_ts.SetTypeName("pb:" + protobuf_type.DESCRIPTOR.full_name)
    aimrt_ts.SetSerializationTypesSupportedList(["pb", "json"])

    sig = inspect.signature(callback)
    params = list(sig.parameters.values())

    if len(params) == 1:
        def handle_callback(serialization_type: str, msg_buf: bytes):
            try:
                msg = DeserializeProtobufMessage(msg_buf, serialization_type, protobuf_type)
                callback(msg)
            except Exception as e:
                print(f"AimRT channel handle get exception, {e}")
        subscriber.SubscribeWithSerializationType(aimrt_ts, handle_callback)
    elif len(params) == 2:
        def handle_callback(ctx_ref: aimrt_python_runtime.ContextRef, msg_buf: bytes):
            try:
                msg = DeserializeProtobufMessage(msg_buf, ctx_ref.GetSerializationType(), protobuf_type)
                callback(ctx_ref, msg)
            except Exception as e:
                print(f"AimRT channel handle get exception, {e}")
        subscriber.SubscribeWithCtx(aimrt_ts, handle_callback)
    else:
        raise TypeError("Invalid callback: expected 1 or 2 parameters")
