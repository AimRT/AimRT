#!/bin/bash

curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.example.ExampleService/GetFooData' \
    -d '{"msg": "test msg"}'
