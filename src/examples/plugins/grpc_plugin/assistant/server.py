# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import argparse
from concurrent import futures

import grpc
from rpc_pb2 import GetBarDataReq, GetBarDataRsp, GetFooDataReq, GetFooDataRsp
from rpc_pb2_grpc import ExampleServiceServicer, add_ExampleServiceServicer_to_server


class ExampleServiceImpl(ExampleServiceServicer):
    def GetFooData(self, request: GetFooDataReq, _) -> GetFooDataRsp:
        print(f"GetFooData: {request.msg}")
        return GetFooDataRsp(msg=f"echo {request.msg}")

    def GetBarData(self, request: GetBarDataReq, _) -> GetBarDataRsp:
        print(f"GetBarData: {request.msg}")
        return GetBarDataRsp(msg=f"echo {request.msg}")


def serve(port: int):
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    add_ExampleServiceServicer_to_server(ExampleServiceImpl(), server)
    server.add_insecure_port(f"[::]:{port}")
    server.start()
    print(f"Server listening on port {port}")
    server.wait_for_termination()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=50050, help="Server port for the service")
    args = parser.parse_args()
    serve(args.port)
