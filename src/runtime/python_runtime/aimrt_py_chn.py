# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import inspect
import sys
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


def _CreateContextRef(ctx_or_type, default_serialization_type: str) -> aimrt_python_runtime.ContextRef:
    if isinstance(ctx_or_type, aimrt_python_runtime.Context):
        ctx_ref = aimrt_python_runtime.ContextRef(ctx_or_type)
    elif isinstance(ctx_or_type, aimrt_python_runtime.ContextRef):
        ctx_ref = ctx_or_type
    else:
        serialization_type = ctx_or_type if isinstance(ctx_or_type, str) else default_serialization_type
        ctx = aimrt_python_runtime.Context()
        ctx.SetSerializationType(serialization_type)
        ctx_ref = aimrt_python_runtime.ContextRef(ctx)

    if ctx_ref.GetSerializationType() == "":
        ctx_ref.SetSerializationType(default_serialization_type)

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
        py_pb_ts = aimrt_python_runtime.PyPbTypeSupport()
        py_pb_ts.SetTypeName("pb:" + msg_type.DESCRIPTOR.full_name)
        py_pb_ts.SetSerializationTypesSupportedList(["pb", "json"])
        return publisher.RegisterPbPublishType(py_pb_ts)
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
    - Publish(publisher, msg)
    - Publish(publisher, msg, ctx)
    - Publish(publisher, msg, serialization_type)
    - Publish(publisher, ctx, msg)
    - Publish(publisher, serialization_type, msg)

    msg: google.protobuf.message.Message | Ros2MsgType
    ctx: aimrt_python_runtime.Context or aimrt_python_runtime.ContextRef
    serialization_type: str

    Args:
        publisher (aimrt_python_runtime.PublisherRef): channel publisher
        second: msg or ctx or serialization_type
        third: msg or ctx or serialization_type or None
    Raises:
        TypeError: if the arguments are invalid
    """
    if isinstance(second, google.protobuf.message.Message):
        msg = second
        ctx_or_type = third
        message_type = "pb"
    elif isinstance(third, google.protobuf.message.Message):
        msg = third
        ctx_or_type = second
        message_type = "pb"
    elif check_is_valid_ros2_msg_type(second):
        msg = second
        ctx_or_type = third
        message_type = "ros2"
    elif check_is_valid_ros2_msg_type(third):
        msg = third
        ctx_or_type = second
        message_type = "ros2"
    else:
        raise TypeError("Invalid arguments, no protobuf message or ROS2 message found")

    if not isinstance(ctx_or_type, (aimrt_python_runtime.Context, aimrt_python_runtime.ContextRef, str)) and \
            ctx_or_type is not None:
        raise TypeError(
            f"Invalid context type: {type(ctx_or_type)}, "
            f"only 'aimrt_python_runtime.Context' or 'aimrt_python_runtime.ContextRef' or 'str' is supported")

    if message_type == "pb":
        ctx_ref = _CreateContextRef(ctx_or_type, default_serialization_type="pb")
        serialized_msg = _SerializeProtobufMessage(msg, ctx_ref.GetSerializationType())
        publisher.PublishPbMessageWithCtx(f"pb:{msg.DESCRIPTOR.full_name}", ctx_ref, serialized_msg)
    elif message_type == "ros2":
        ctx_ref = _CreateContextRef(ctx_or_type, default_serialization_type="ros2")
        publisher.PublishRos2MessageWithCtx("ros2:" + msg.__class__.__name__, ctx_ref, msg)


def Subscribe(subscriber: aimrt_python_runtime.SubscriberRef,
              msg_type: google.protobuf.message.Message | Ros2MsgType,
              callback: Callable):
    """Subscribe a message from a channel.

    Args:
        subscriber (aimrt_python_runtime.SubscriberRef): channel subscriber
        msg_type: protobuf message type or ROS2 message type
        callback (Callable): callback function

    Raises:
        ValueError: if the callback is invalid
        TypeError: if the message type is invalid

    Callback function signature (both for Protobuf and ROS2 messages):
    - callback(msg)
    - callback(ctx, msg)
    """
    # Check callback signature
    sig = inspect.signature(callback)
    required_param_count = sum(1 for param in sig.parameters.values() if param.default == param.empty)

    if not (1 <= required_param_count <= 2):
        raise ValueError("Invalid callback: expected 1 or 2 parameters, with at most one optional parameter")

    if isinstance(msg_type, google._upb._message.MessageMeta):
        py_pb_ts = aimrt_python_runtime.PyPbTypeSupport()
        py_pb_ts.SetTypeName("pb:" + msg_type.DESCRIPTOR.full_name)
        py_pb_ts.SetSerializationTypesSupportedList(["pb", "json"])

        def handle_callback(ctx_ref: aimrt_python_runtime.ContextRef, msg_buf: bytes):
            try:
                msg = _DeserializeProtobufMessage(msg_buf, ctx_ref.GetSerializationType(), msg_type)
                if required_param_count == 1:
                    callback(msg)
                else:
                    callback(ctx_ref, msg)
            except Exception as e:
                print(f"AimRT channel handle get exception, {e}", file=sys.stderr)

        subscriber.SubscribePbMessageWithCtx(py_pb_ts, handle_callback)

    elif check_is_valid_ros2_msg_type(msg_type):
        py_ros2_ts = aimrt_python_runtime.PyRos2TypeSupport(msg_type)
        py_ros2_ts.SetTypeName("ros2:" + msg_type.__name__)
        py_ros2_ts.SetSerializationTypesSupportedList(["ros2"])

        def ros2_callback_wrapper(ctx_ref: aimrt_python_runtime.ContextRef, msg):
            try:
                if required_param_count == 1:
                    callback(msg)
                else:
                    callback(ctx_ref, msg)
            except Exception as e:
                print(f"AimRT channel handle get exception, {e}", file=sys.stderr)

        subscriber.SubscribeRos2MessageWithCtx(py_ros2_ts, msg_type, ros2_callback_wrapper)

    else:
        raise TypeError(f"Invalid message type: {type(msg_type)}, expected Protobuf or ROS2 message type")
