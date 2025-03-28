#!/bin/bash

protocols_dir=../../../protocols/ros2/example_ros2/

aimrt_py-gen-ros2-rpc --pkg_name=example_ros2 --srv_file=${protocols_dir}/RosTestRpc.srv --output_path=./
