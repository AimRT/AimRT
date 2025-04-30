#!/bin/bash

data='{
    "action_name": "my_signal_record",
    "kv_pairs":{ "key": "timestamp: '2023-10-25T12:34:56.789Z'\nposition:\n  x: 1.2\n  y: 3.4\n  z: 0.0\norientation:\n  roll: 0.0\n  pitch: 0.0\n  yaw: 1.57\nsensor_temperature: 25.5C\nsensor_distance_front: 1.8m\nbattery_voltage: 12.4V\nbattery_level: 85%\nmotor_speed_left: 100rpm\nmotor_speed_right: 102rpm\nstatus: active\nmode: autonomous\nlog_message: Navigation started."}
}'
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.record_playback_plugin.RecordPlaybackService/UpdateMetadata' \
    -d "$data"