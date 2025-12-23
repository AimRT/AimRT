#!/bin/bash

data='{
    "action_names": []
}'
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.record_playback_plugin.RecordPlaybackService/GetRecordActionStatus' \
    -d "$data"
