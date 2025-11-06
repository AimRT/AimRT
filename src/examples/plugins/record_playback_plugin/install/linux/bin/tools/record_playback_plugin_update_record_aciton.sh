#!/bin/bash

data='{
    "action_name": "my_signal_record",
    "topic_metas": [
        {
            "topic_name": "test_topic",
            "msg_type": "pb:aimrt.protocols.example.ExampleEventMsg",
            "record_enabled": false
        },
    ]
}'
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.record_playback_plugin.RecordPlaybackService/UpdateRecordAction' \
    -d "$data"