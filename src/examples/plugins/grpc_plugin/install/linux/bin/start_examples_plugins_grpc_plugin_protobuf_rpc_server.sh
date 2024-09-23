#!/bin/bash

lsof -i :50050 | awk 'NR!=1 {print $2}' | xargs kill -9

./aimrt_main --cfg_file_path=./cfg/examples_plugins_grpc_plugin_protobuf_rpc_server_cfg.yaml
