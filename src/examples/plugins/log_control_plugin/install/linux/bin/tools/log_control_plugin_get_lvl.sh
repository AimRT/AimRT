#!/bin/bash

curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.log_control_plugin.LogControlService/GetModuleLogLevel' \
    -d '{"module_names": []}'
