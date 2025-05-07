from __future__ import annotations
from aimrt_py import aimrt_python_runtime
import aimrt_py.aimrt_python_runtime
from aimrt_py.check_ros2_type import check_is_valid_ros2_msg_type
import google as google
import inspect as inspect
import sys as sys
import typing
from typing import TypeVar
__all__ = ['GetPbMessageTypeName', 'GetRos2MessageTypeName', 'Publish', 'RegisterPublishType', 'Ros2MsgType', 'Subscribe', 'TypeVar', 'aimrt_python_runtime', 'check_is_valid_ros2_msg_type', 'google', 'inspect', 'sys']
def GetPbMessageTypeName(msg: google._upb._message.MessageMeta) -> str:
    ...
def GetRos2MessageTypeName(msg_type: ~Ros2MsgType) -> str:
    ...
def Publish(publisher: aimrt_py.aimrt_python_runtime.PublisherRef, second, third = None):
    """
    Publish a message to a channel.
    
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
def RegisterPublishType(publisher: aimrt_py.aimrt_python_runtime.PublisherRef, msg_type: typing.Union[google._upb._message.MessageMeta, ~Ros2MsgType]):
    """
    Register a protobuf message type to a publisher.
    
        Args:
            publisher (aimrt_python_runtime.PublisherRef): channel publisher
            msg_type (google.protobuf.message.Message | Ros2MsgType): protobuf message type or ROS2 message type
    
        Returns:
            bool: True if success, False otherwise
        
    """
def Subscribe(subscriber: aimrt_py.aimrt_python_runtime.SubscriberRef, msg_type: typing.Union[google._upb._message.MessageMeta, ~Ros2MsgType], callback: typing.Callable):
    """
    Subscribe a message from a channel.
    
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
def _CreateContextRef(ctx_or_type, default_serialization_type: str) -> aimrt_py.aimrt_python_runtime.ContextRef:
    ...
def _DeserializeProtobufMessage(msg_buf: bytes, serialization_type: str, protobuf_type: google.protobuf.message.Message) -> google.protobuf.message.Message:
    ...
def _SerializeProtobufMessage(pb_msg: google.protobuf.message.Message, serialization_type: str) -> bytes:
    ...
Ros2MsgType: typing.TypeVar  # value = ~Ros2MsgType
