#!/bin/bash

curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.parameter_plugin.ParameterService/Set' \
    -d '{"module_name": "ParameterModule", "parameter_key": "key-1", "parameter_value": "val-abc"}'
