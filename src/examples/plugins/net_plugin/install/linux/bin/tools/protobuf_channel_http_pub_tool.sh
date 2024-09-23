#!/bin/bash

curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/channel/test_topic/pb%3Aaimrt.protocols.example.ExampleEventMsg' \
    -d '{"msg": "test msg", "num": 123}'
