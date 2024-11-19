from typing import overload

import aimrt_py
import example_ros2.srv

if example_ros2.srv.RosTestRpc._TYPE_SUPPORT is None:
    example_ros2.srv.RosTestRpc.__import_type_support__()


class ExampleRos2Service(aimrt_py.ServiceBase):
    def __init__(self):
        super().__init__("ros2", "example_ros2/srv")

        # RosTestRpc
        ros_test_rpc_req_aimrt_ts = aimrt_py.PyRos2TypeSupport(example_ros2.srv.RosTestRpc.Request)
        ros_test_rpc_req_aimrt_ts.SetTypeName(aimrt_py.GetRos2MessageTypeName(example_ros2.srv.RosTestRpc.Request))
        ros_test_rpc_req_aimrt_ts.SetSerializationTypesSupportedList(["ros2"])

        ros_test_rpc_rsp_aimrt_ts = aimrt_py.PyRos2TypeSupport(example_ros2.srv.RosTestRpc.Response)
        ros_test_rpc_rsp_aimrt_ts.SetTypeName(aimrt_py.GetRos2MessageTypeName(example_ros2.srv.RosTestRpc.Response))
        ros_test_rpc_rsp_aimrt_ts.SetSerializationTypesSupportedList(["ros2"])

        if not aimrt_py.check_is_valid_srv_type(example_ros2.srv.RosTestRpc):
            raise RuntimeError("The service type provided is not valid")

        self.Ros2RegisterServiceFunc(
            "RosTestRpc",
            example_ros2.srv.RosTestRpc,
            ros_test_rpc_req_aimrt_ts,
            example_ros2.srv.RosTestRpc.Request,
            ros_test_rpc_rsp_aimrt_ts,
            example_ros2.srv.RosTestRpc.Response,
            self.RosTestRpc)

    def RosTestRpc(self, ctx_ref, req):
        return aimrt_py.RpcStatus(aimrt_py.RpcStatusRetCode.SVR_NOT_IMPLEMENTED)


class ExampleRos2ServiceProxy(aimrt_py.ProxyBase):
    def __init__(self, rpc_handle_ref=aimrt_py.RpcHandleRef()):
        super().__init__(rpc_handle_ref, "ros2", "example_ros2")
        self.rpc_handle_ref = rpc_handle_ref

    @overload
    def RosTestRpc(
        self, req: example_ros2.srv.RosTestRpc.Request
    ) -> tuple[aimrt_py.RpcStatus, example_ros2.srv.RosTestRpc.Response]: ...

    @overload
    def RosTestRpc(
        self, ctx: aimrt_py.RpcContext, req: example_ros2.srv.RosTestRpc.Request
    ) -> tuple[aimrt_py.RpcStatus, example_ros2.srv.RosTestRpc.Response]: ...

    @overload
    def RosTestRpc(
        self, ctx_ref: aimrt_py.RpcContextRef, req: example_ros2.srv.RosTestRpc.Request
    ) -> tuple[aimrt_py.RpcStatus, example_ros2.srv.RosTestRpc.Response]: ...


    def RosTestRpc(self, *args) -> tuple[aimrt_py.RpcStatus, example_ros2.srv.RosTestRpc.Response]:
        if len(args) == 1:
            ctx = super().NewContextSharedPtr()
            req = args[0]
        elif len(args) == 2:
            ctx = args[0]
            req = args[1]
        else:
            raise TypeError(f"GetFooData expects 1 or 2 arguments, got {len(args)}")

        if isinstance(ctx, aimrt_py.RpcContext):
            ctx_ref = aimrt_py.RpcContextRef(ctx)
        elif isinstance(ctx, aimrt_py.RpcContextRef):
            ctx_ref = ctx
        else:
            raise TypeError(f"ctx must be 'aimrt_py.RpcContext' or 'aimrt_py.RpcContextRef', got {type(ctx)}")

        if ctx_ref:
            if ctx_ref.GetSerializationType() == "":
                ctx_ref.SetSerializationType("ros2")
        else:
            real_ctx = aimrt_py.RpcContext()
            ctx_ref = aimrt_py.RpcContextRef(real_ctx)
            ctx_ref.SetSerializationType("ros2")

        status, rsp = self.rpc_handle_ref.Ros2Invoke(
            "ros2:/example_ros2/srv/RosTestRpc", ctx_ref, req, example_ros2.srv.RosTestRpc.Response)

        return status, rsp

    @staticmethod
    def RegisterClientFunc(rpc_handle):

        ros_test_rpc_req_aimrt_ts = aimrt_py.PyRos2TypeSupport(example_ros2.srv.RosTestRpc.Request)
        ros_test_rpc_req_aimrt_ts.SetTypeName(aimrt_py.GetRos2MessageTypeName(example_ros2.srv.RosTestRpc.Request))
        ros_test_rpc_req_aimrt_ts.SetSerializationTypesSupportedList(["ros2"])

        ros_test_rpc_rsp_aimrt_ts = aimrt_py.PyRos2TypeSupport(example_ros2.srv.RosTestRpc.Response)
        ros_test_rpc_rsp_aimrt_ts.SetTypeName(aimrt_py.GetRos2MessageTypeName(example_ros2.srv.RosTestRpc.Response))
        ros_test_rpc_rsp_aimrt_ts.SetSerializationTypesSupportedList(["ros2"])

        return rpc_handle.Ros2RegisterClientFunc(
            "ros2:/example_ros2/srv/RosTestRpc",
            example_ros2.srv.RosTestRpc,
            ros_test_rpc_req_aimrt_ts,
            ros_test_rpc_rsp_aimrt_ts)
