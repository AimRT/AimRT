# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import argparse
import datetime
import threading
import time

import aimrt_py
import rpc_aimrt_rpc_pb2
import rpc_pb2
from google.protobuf.json_format import MessageToJson


def main():
    parser = argparse.ArgumentParser(description='Example rpc client app.')
    parser.add_argument('--cfg_file_path', type=str, default="", help='config file path')
    args = parser.parse_args()

    print("AimRT start.")

    aimrt_core = aimrt_py.Core()

    # Initialize
    core_options = aimrt_py.CoreOptions()
    core_options.cfg_file_path = args.cfg_file_path
    aimrt_core.Initialize(core_options)

    # Create Module
    module_handle = aimrt_core.CreateModule("NormalRpcClientPyModule")

    # Register rpc client
    rpc_handle = module_handle.GetRpcHandle()
    ret = rpc_aimrt_rpc_pb2.ExampleServiceProxy.RegisterClientFunc(rpc_handle)
    assert ret, "Register client failed."

    # Start
    thread = threading.Thread(target=aimrt_core.Start)
    thread.start()

    # Sleep for seconds
    time.sleep(1)

    # Call rpc
    proxy = rpc_aimrt_rpc_pb2.ExampleServiceProxy(rpc_handle)

    req = rpc_pb2.GetFooDataReq()
    req.msg = "example msg"

    ctx = aimrt_py.RpcContext()
    ctx.SetTimeout(datetime.timedelta(seconds=30))
    ctx.SetMetaValue("key1", "value1")
    status, rsp = proxy.GetFooData(ctx, req)

    aimrt_py.info(module_handle.GetLogger(),
                  f"Call rpc done, status: {status.ToString()}, "
                  f"req: {MessageToJson(req)}, "
                  f"rsp: {MessageToJson(rsp)}")

    # Shutdown
    aimrt_core.Shutdown()

    thread.join()

    print("AimRT exit.")


if __name__ == '__main__':
    main()
