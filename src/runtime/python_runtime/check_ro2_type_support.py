# Copyright (c) 2024 The AimRT Authors.
# AimRT is licensed under Mulan PSL v2.

def check_for_ros2_type_support(msg_or_srv_type):
    try:
        ts = msg_or_srv_type.__class__._TYPE_SUPPORT
    except AttributeError as e:
        e.args = (
            e.args[0] +
            ' This might be a ROS 1 message type but it should be a ROS 2 message type.'
            ' Make sure to source your ROS 2 workspace after your ROS 1 workspace.',
            *e.args[1:])
        return False
    if ts is None:
        msg_or_srv_type.__class__.__import_type_support__()
    return msg_or_srv_type.__class__._TYPE_SUPPORT is None


def check_is_valid_ros2_msg_type(msg_type):
    if not check_for_ros2_type_support(msg_type):
        return False
    try:
        assert None not in (
            msg_type.__class__._CREATE_ROS_MESSAGE,
            msg_type.__class__._CONVERT_FROM_PY,
            msg_type.__class__._CONVERT_TO_PY,
            msg_type.__class__._DESTROY_ROS_MESSAGE,
        )
    except (AssertionError, AttributeError):
        return False
    return True
