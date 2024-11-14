import aimrt_py
import example_ros2.srv

if example_ros2.srv.RosTestRpc._TYPE_SUPPORT is None:
    example_ros2.srv.RosTestRpc.__import_type_support__()

req_type = example_ros2.srv.RosTestRpc.Request
rsp_type = example_ros2.srv.RosTestRpc.Response


class ExampleRos2Service(aimrt_py.ServiceBase):
    def __init__(self):
        super().__init__("ros2", "example_ros2/srv")

        # RosTestRpc
        RosTestRpcReq_aimrt_ts = aimrt_py.PyRos2TypeSupport(req_type)
        RosTestRpcReq_aimrt_ts.SetTypeName(aimrt_py.GetRos2MessageTypeName(req_type))
        RosTestRpcReq_aimrt_ts.SetSerializationTypesSupportedList(["ros2"])

        RosTestRpcRsp_aimrt_ts = aimrt_py.PyRos2TypeSupport(rsp_type)
        RosTestRpcRsp_aimrt_ts.SetTypeName(aimrt_py.GetRos2MessageTypeName(rsp_type))
        RosTestRpcRsp_aimrt_ts.SetSerializationTypesSupportedList(["ros2"])

        self.Ros2RegisterServiceFunc(
            "RosTestRpc",
            RosTestRpcReq_aimrt_ts,
            req_type,
            RosTestRpcRsp_aimrt_ts,
            rsp_type,
            self.RosTestRpc)

    def RosTestRpc(self, ctx_ref, req):
        return aimrt_py.RpcStatus(aimrt_py.RpcStatusRetCode.SVR_NOT_IMPLEMENTED)


class ExampleRos2ServiceProxy(aimrt_py.ProxyBase):
    def __init__(self, rpc_handle_ref=aimrt_py.RpcHandleRef()):
        super().__init__(rpc_handle_ref, "ros2", "example_ros2")
        self.rpc_handle_ref = rpc_handle_ref

    def RosTestRpc(self, ctx_ref, req):
        rsp = rsp_type()

        status, rsp_ret = self.rpc_handle_ref.Ros2Invoke("ros2:/example_ros2/srv/RosTestRpc", ctx_ref, req, rsp)

        return status, rsp_ret

    @staticmethod
    def RegisterClientFunc(rpc_handle):

        RosTestRpcReq_aimrt_ts = aimrt_py.PyRos2TypeSupport(req_type)
        RosTestRpcReq_aimrt_ts.SetTypeName(aimrt_py.GetRos2MessageTypeName(req_type))
        RosTestRpcReq_aimrt_ts.SetSerializationTypesSupportedList(["ros2"])

        RosTestRpcRsp_aimrt_ts = aimrt_py.PyRos2TypeSupport(rsp_type)
        RosTestRpcRsp_aimrt_ts.SetTypeName(aimrt_py.GetRos2MessageTypeName(rsp_type))
        RosTestRpcRsp_aimrt_ts.SetSerializationTypesSupportedList(["ros2"])

        return rpc_handle.Ros2RegisterClientFunc(
            "ros2:/example_ros2/srv/RosTestRpc",
            RosTestRpcReq_aimrt_ts,
            RosTestRpcRsp_aimrt_ts)
