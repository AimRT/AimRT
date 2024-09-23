#!/bin/bash

curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/channel/test_topic/ros2%3Aexample_ros2%2Fmsg%2FRosTestMsg' \
    -d '{"num": 123, "data": [1, 2, 3]}'
