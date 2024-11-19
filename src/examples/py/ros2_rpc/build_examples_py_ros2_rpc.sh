#!/bin/bash

protocols_dir=../../../protocols/example_ros2/

aimrt_py-gen-ros2-rpc --package_name=example_ros2 --service_file=${protocols_dir}/RosTestRpc.srv --output_path=./
