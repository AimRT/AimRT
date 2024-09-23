#!/bin/bash

curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.time_manipulator_plugin.TimeManipulatorService/GetTimeRatio' \
    -d '{"executor_name": "time_schedule_executor"}'
