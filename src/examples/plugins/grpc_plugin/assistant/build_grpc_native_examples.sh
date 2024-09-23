#!/bin/bash

set -ex

# Check if grpc_tools is installed
if ! python3 -c "import grpc_tools" &> /dev/null; then
    echo "Error: grpc_tools is not installed. Please run 'pip install grpcio-tools' to install it."
    exit 1
fi

protocols_dir=../../../../protocols/example/

# Generate only protobuf code for common.proto
python3 -m grpc_tools.protoc \
    -I${protocols_dir} \
    --python_out=./ \
    ${protocols_dir}/common.proto

# Generate both protobuf and gRPC code for rpc.proto
python3 -m grpc_tools.protoc \
    -I${protocols_dir} \
    --python_out=./ \
    --grpc_python_out=./ \
    ${protocols_dir}/rpc.proto

echo "Code generation completed."
