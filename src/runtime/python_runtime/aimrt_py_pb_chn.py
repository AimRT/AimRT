# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import inspect
from typing import Callable

import google.protobuf
import google.protobuf.message

from . import aimrt_python_runtime


def _SerializeProtobufMessage(pb_msg: google.protobuf.message.Message, serialization_type: str) -> bytes:
    if serialization_type == "pb":
        return pb_msg.SerializeToString()
    elif serialization_type == "json":
        return google.protobuf.json_format.MessageToJson(pb_msg)
    else:
        raise ValueError(f"Invalid serialization type: {serialization_type}")


def _DeserializeProtobufMessage(msg_buf: bytes,
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


def RegisterPublishType(publisher: aimrt_python_runtime.PublisherRef, protobuf_type: google.protobuf.message.Message):
    """Register a protobuf message type to a publisher.

    Args:
        publisher (aimrt_python_runtime.PublisherRef): channel publisher
        protobuf_type (google.protobuf.message.Message): protobuf message type

    Returns:
        bool: True if success, False otherwise
    """
    aimrt_ts = aimrt_python_runtime.TypeSupport()
    aimrt_ts.SetTypeName("pb:" + protobuf_type.DESCRIPTOR.full_name)
    aimrt_ts.SetSerializationTypesSupportedList(["pb", "json"])

    return publisher.RegisterPublishType(aimrt_ts)


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
        if isinstance(ctx, aimrt_python_runtime.Context):
            ctx_ref = aimrt_python_runtime.ContextRef(ctx)
        else:
            ctx_ref = ctx
        if ctx_ref.GetSerializationType() == "":
            ctx_ref.SetSerializationType("pb")
        serialized_msg = _SerializeProtobufMessage(pb_msg, ctx_ref.GetSerializationType())
        publisher.PublishWithCtx(f"pb:{pb_msg.DESCRIPTOR.full_name}", ctx_ref, serialized_msg)
    elif isinstance(ctx, str):
        serialization_type = ctx
        serialized_msg = _SerializeProtobufMessage(pb_msg, serialization_type)
        publisher.PublishWithSerializationType(f"pb:{pb_msg.DESCRIPTOR.full_name}", serialization_type, serialized_msg)
    elif ctx is None:
        # default use pb serialization
        serialized_msg = _SerializeProtobufMessage(pb_msg, "pb")
        publisher.PublishWithSerializationType(f"pb:{pb_msg.DESCRIPTOR.full_name}", "pb", serialized_msg)
    else:
        raise TypeError(
            f"Invalid context type: {type(ctx)}, "
            f"only 'aimrt_python_runtime.Context' or 'aimrt_python_runtime.ContextRef' or 'str' is supported")


def Subscribe(subscriber: aimrt_python_runtime.SubscriberRef,
              protobuf_type: google.protobuf.message.Message,
              callback: Callable):
    """Subscribe a message from a channel.

    Args:
        subscriber (aimrt_python_runtime.SubscriberRef): channel subscriber
        protobuf_type (google.protobuf.message.Message): protobuf message type
        callback (Callable): callback function

    Raises:
        ValueError: if the callback is invalid

    Callback function signature:
    - callback(msg)
    - callback(ctx, msg)
    """
    aimrt_ts = aimrt_python_runtime.TypeSupport()
    aimrt_ts.SetTypeName("pb:" + protobuf_type.DESCRIPTOR.full_name)
    aimrt_ts.SetSerializationTypesSupportedList(["pb", "json"])

    sig = inspect.signature(callback)
    required_param_count = sum(1 for param in sig.parameters.values() if param.default == param.empty)

    if not (1 <= required_param_count <= 2):
        raise ValueError("Invalid callback: expected 1 or 2 parameters, with at most one optional parameter")

    def handle_callback(ctx_ref: aimrt_python_runtime.ContextRef, msg_buf: bytes):
        try:
            msg = _DeserializeProtobufMessage(msg_buf, ctx_ref.GetSerializationType(), protobuf_type)
            if required_param_count == 1:
                callback(msg)
            else:
                callback(ctx_ref, msg)
        except Exception as e:
            print(f"AimRT channel handle get exception, {e}")

    subscriber.SubscribeWithCtx(aimrt_ts, handle_callback)
