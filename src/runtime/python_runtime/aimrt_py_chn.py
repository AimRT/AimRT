# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import inspect
from typing import Callable, TypeVar

import google._upb._message
import google.protobuf
import google.protobuf.message

from . import aimrt_python_runtime
from .check_ros2_msg_type import check_is_valid_ros2_msg_type

Ros2MsgType = TypeVar("Ros2MsgType")


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


def _CreateContextRef(ctx_or_type) -> aimrt_python_runtime.ContextRef:
    if isinstance(ctx_or_type, aimrt_python_runtime.Context):
        ctx_ref = aimrt_python_runtime.ContextRef(ctx_or_type)
    elif isinstance(ctx_or_type, aimrt_python_runtime.ContextRef):
        ctx_ref = ctx_or_type
    else:
        serialization_type = ctx_or_type if isinstance(ctx_or_type, str) else "pb"
        ctx = aimrt_python_runtime.Context()
        ctx.SetSerializationType(serialization_type)
        ctx_ref = aimrt_python_runtime.ContextRef(ctx)

    if ctx_ref.GetSerializationType() == "":
        ctx_ref.SetSerializationType("pb")

    return ctx_ref


def RegisterPublishType(publisher: aimrt_python_runtime.PublisherRef,
                        msg_type: google._upb._message.MessageMeta | Ros2MsgType):
    """Register a protobuf message type to a publisher.

    Args:
        publisher (aimrt_python_runtime.PublisherRef): channel publisher
        msg_type (google.protobuf.message.Message | Ros2MsgType): protobuf message type or ROS2 message type

    Returns:
        bool: True if success, False otherwise
    """
    if isinstance(msg_type, google._upb._message.MessageMeta):
        aimrt_ts = aimrt_python_runtime.TypeSupport()
        aimrt_ts.SetTypeName("pb:" + msg_type.DESCRIPTOR.full_name)
        aimrt_ts.SetSerializationTypesSupportedList(["pb", "json"])
        return publisher.RegisterPbPublishType(aimrt_ts)
    elif check_is_valid_ros2_msg_type(msg_type):
        py_ros2_ts = aimrt_python_runtime.PyRos2TypeSupport(msg_type)
        py_ros2_ts.SetTypeName("ros2:" + msg_type.__name__)
        py_ros2_ts.SetSerializationTypesSupportedList(["ros2"])
        return publisher.RegisterRos2PublishType(py_ros2_ts)
    else:
        raise TypeError(f"Invalid message type: {type(msg_type)}")


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
        ctx_or_type = third
    elif isinstance(third, google.protobuf.message.Message):
        pb_msg = third
        ctx_or_type = second
    else:
        raise TypeError("Invalid arguments, no protobuf message found")

    if isinstance(ctx_or_type, (aimrt_python_runtime.Context, aimrt_python_runtime.ContextRef, str)) or \
            ctx_or_type is None:
        ctx_ref = _CreateContextRef(ctx_or_type)
        serialized_msg = _SerializeProtobufMessage(pb_msg, ctx_ref.GetSerializationType())
        publisher.PublishPbMessageWithCtx(f"pb:{pb_msg.DESCRIPTOR.full_name}", ctx_ref, serialized_msg)
    else:
        raise TypeError(
            f"Invalid context type: {type(ctx_or_type)}, "
            f"only 'aimrt_python_runtime.Context' or 'aimrt_python_runtime.ContextRef' or 'str' is supported")


def PublishRos2Message(publisher: aimrt_python_runtime.PublisherRef, msg: Ros2MsgType):
    ctx = aimrt_python_runtime.Context()
    ctx.SetSerializationType("ros2")
    ctx_ref = aimrt_python_runtime.ContextRef(ctx)
    publisher.PublishPbMessageWithCtx("ros2:" + msg.__class__.__name__, ctx_ref, msg)


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


def SubscribeRos2Message(subscriber: aimrt_python_runtime.SubscriberRef,
                         ros2_msg_type: Ros2MsgType,
                         callback: Callable):
    if not check_is_valid_ros2_msg_type(ros2_msg_type):
        raise TypeError(f"Invalid ROS2 message type: {ros2_msg_type}")

    py_ros2_ts = aimrt_python_runtime.PyRos2TypeSupport(ros2_msg_type)
    py_ros2_ts.SetTypeName("ros2:" + ros2_msg_type.__name__)
    py_ros2_ts.SetSerializationTypesSupportedList(["ros2"])

    subscriber.SubscribeRos2MessageWithCtx(py_ros2_ts, ros2_msg_type, callback)
