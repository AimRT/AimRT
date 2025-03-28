#!/bin/bash

protoc_cmd=protoc
protocols_dir=../../../protocols/pb/example/

if ! command -v ${protoc_cmd} &> /dev/null; then
    echo "Can not find protoc!"
    exit 1
fi

PROTOC_VERSION=$(${protoc_cmd} --version | awk '{print $2}')

version_greater_equal() {
    local version1=$1
    local version2=$2

    [ "$(printf '%s\n' "$version1" "$version2" | sort -V | head -n1)" != "$version1" ]
}

if version_greater_equal "$PROTOC_VERSION" "3.20"; then
    echo "protoc version $PROTOC_VERSION."
else
    echo "protoc version $PROTOC_VERSION, need 3.20 at least."
    exit 1
fi

${protoc_cmd} -I${protocols_dir} --python_out=./ ${protocols_dir}/common.proto ${protocols_dir}/rpc.proto
${protoc_cmd} -I${protocols_dir} --aimrt_rpc_out=./ --plugin=protoc-gen-aimrt_rpc=$(which protoc_plugin_py_gen_aimrt_py_rpc) ${protocols_dir}/rpc.proto
