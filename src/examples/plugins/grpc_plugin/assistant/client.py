# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import argparse

import grpc
from rpc_pb2 import GetBarDataReq, GetFooDataReq
from rpc_pb2_grpc import ExampleServiceStub


class ExampleServiceClient:
    def __init__(self, channel):
        self.stub = ExampleServiceStub(channel)

    def get_foo_data(self, msg):
        request = GetFooDataReq(msg=msg)
        try:
            response = self.stub.GetFooData(request)
            return response.msg
        except grpc.RpcError as e:
            return f"RPC failed. error code: {e.code()}, error message: {e.details()}"

    def get_bar_data(self, msg):
        request = GetBarDataReq(msg=msg)
        try:
            response = self.stub.GetBarData(request)
            return response.msg
        except grpc.RpcError as e:
            return f"RPC failed. error code: {e.code()}, error message: {e.details()}"


def main():
    parser = argparse.ArgumentParser(description="gRPC client example")
    parser.add_argument("--target", default="localhost:50050", help="Server address")
    args = parser.parse_args()

    with grpc.insecure_channel(args.target) as channel:
        client = ExampleServiceClient(channel)

        long_str = "a" * 100
        print(f"GetFooData response: {client.get_foo_data('Hello Foo')}")
        print(f"GetBarData response: {client.get_bar_data(long_str)}")


if __name__ == "__main__":
    main()
