#!/bin/bash

lsof -i :50051 | awk 'NR!=1 {print $2}' | xargs kill -9

./aimrt_main --cfg_file_path=./cfg/examples_plugins_grpc_plugin_pb_rpc_client_cfg.yaml
