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

        self.Ros2RegisterServiceFunc(
            "RosTestRpc",
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

    def RosTestRpc(self, ctx_ref, req):
        rsp = example_ros2.srv.RosTestRpc.Response()

        status, rsp_ret = self.rpc_handle_ref.Ros2Invoke("ros2:/example_ros2/srv/RosTestRpc", ctx_ref, req, rsp)

        return status, rsp_ret

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
            ros_test_rpc_req_aimrt_ts,
            ros_test_rpc_rsp_aimrt_ts)
