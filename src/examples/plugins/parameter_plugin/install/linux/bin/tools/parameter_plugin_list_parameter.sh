#!/bin/bash

curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.parameter_plugin.ParameterService/List' \
    -d '{"module_name": "ParameterModule"}'
