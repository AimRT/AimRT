# Copyright (c) 2024 The AimRT Authors.
# AimRT is licensed under Mulan PSL v2.

import datetime
import random
import string
import threading
import time

import aimrt_py
import rpc_aimrt_rpc_pb2
import rpc_pb2
import yaml


class BenchmarkRpcClientModule(aimrt_py.ModuleBase):
    def __init__(self):
        super().__init__()
        self.core: aimrt_py.CoreRef = None
        self.bench_plans: list = []
        self.proxy: rpc_aimrt_rpc_pb2.ExampleServiceProxy = None
        self.client_statistics_executor: aimrt_py.ExecutorRef = None
        self.executor_vec: list[aimrt_py.ExecutorRef] = []
        self.bench_interval: int = 1  # seconds
        self.shutdown_delay: int = 2  # seconds
        self.run_flag: bool = True
        self.stop_sig: threading.Event = threading.Event()

    def Info(self) -> aimrt_py.ModuleInfo:
        info = aimrt_py.ModuleInfo()
        info.name = "BenchmarkRpcClientModule"
        return info

    def Initialize(self, core: aimrt_py.CoreRef) -> bool:
        assert isinstance(core, aimrt_py.CoreRef)

        self.core = core
        self.logger = self.core.GetLogger()

        aimrt_py.info(self.logger, "Module initializing ...")

        try:
            # read config
            file_path = self.core.GetConfigurator().GetConfigFilePath()
            with open(file_path, "r") as f:
                cfg_node = yaml.safe_load(f)

            self.max_parallel = cfg_node.get("max_parallel", 0)

            if "bench_plans" in cfg_node and isinstance(cfg_node["bench_plans"], list):
                for bench_plan_node in cfg_node["bench_plans"]:
                    bench_plan = {
                        "perf_mode": bench_plan_node["perf_mode"],
                        "msg_size": bench_plan_node["msg_size"],
                        "parallel": bench_plan_node["parallel"],
                        "msg_count": bench_plan_node["msg_count"]
                    }

                    if bench_plan["perf_mode"] == "fixed-freq":
                        bench_plan["freq"] = bench_plan_node["freq"]

                    if bench_plan["parallel"] > self.max_parallel:
                        raise ValueError(f"Bench plan parallel ({bench_plan['parallel']}) "
                                         f"is greater than max parallel ({self.max_parallel})")

                    self.bench_plans.append(bench_plan)

            # get rpc handle
            rpc_handle = self.core.GetRpcHandle()
            assert rpc_handle, "Get rpc handle failed."

            # register rpc client
            ret = rpc_aimrt_rpc_pb2.ExampleServiceProxy.RegisterClientFunc(rpc_handle)
            assert ret, "Register client failed."

            # create rpc proxy
            self.proxy = rpc_aimrt_rpc_pb2.ExampleServiceProxy(rpc_handle)

            # check executor
            self.client_statistics_executor = self.core.GetExecutorManager().GetExecutor("client_statistics_executor")
            assert self.client_statistics_executor and self.client_statistics_executor.SupportTimerSchedule(), \
                "Get executor 'client_statistics_executor' failed."

            for ii in range(self.max_parallel):
                executor_name = f"client_executor_{ii}"
                executor = self.core.GetExecutorManager().GetExecutor(executor_name)
                assert executor and executor.SupportTimerSchedule(), f"Get executor '{executor_name}' failed."
                self.executor_vec.append(executor)

            aimrt_py.info(self.logger, f"Module config: max_parallel={self.max_parallel}, "
                                       f"bench_plans={self.bench_plans}")

        except Exception as e:
            aimrt_py.error(self.logger, f"Initialize failed: {e}")
            return False

        aimrt_py.info(self.logger, "Init succeeded")
        return True

    def Start(self) -> bool:
        try:
            aimrt_py.info(self.logger, "Module starting ...")
            self.client_statistics_executor.Execute(self.MainLoop)

        except Exception as e:
            aimrt_py.error(self.logger, f"Start failed: {e}")
            return False

        aimrt_py.info(self.logger, "Start succeeded")
        return True

    def Shutdown(self) -> bool:
        try:
            aimrt_py.info(self.logger, "Module shutting down ...")

            self.run_flag = False
            self.request_complete_event.set()
            self.stop_sig.wait()
        except Exception as e:
            aimrt_py.error(self.logger, f"Shutdown failed: {e}")
            return False

        aimrt_py.info(self.logger, "Shutdown succeeded")
        return True

    def MainLoop(self) -> None:
        try:
            aimrt_py.info(self.logger, "Start Bench.")

            # wait for service server
            self.WaitForServiceServer()

            # benchmark
            for ii, bench_plan in enumerate(self.bench_plans):
                if not self.run_flag:
                    break

                aimrt_py.info(self.logger, f"Start bench plan {ii}")
                self.StartSinglePlan(ii, bench_plan)
                aimrt_py.info(self.logger, f"End bench plan {ii}")

                time.sleep(self.bench_interval)

            aimrt_py.info(self.logger, "Bench completed.")

        except Exception as e:
            aimrt_py.error(self.logger, f"Exit MainLoop with exception: {e}")

        self.stop_sig.set()

    def WaitForServiceServer(self) -> None:
        aimrt_py.debug(self.logger, "wait for service server...")

        req = rpc_pb2.GetFooDataReq()
        req.msg = self.GenerateRandomString(10)

        while self.run_flag:
            ctx = aimrt_py.RpcContext()
            ctx.SetTimeout(datetime.timedelta(seconds=3))
            status, _ = self.proxy.GetFooData(ctx, req)
            if status.Code() != aimrt_py.RpcStatusRetCode.OK:
                aimrt_py.warn(self.logger, "Server is not available!!!")
            else:
                break

            time.sleep(1)

        aimrt_py.debug(self.logger, "Server is available!!!")

    def StartSinglePlan(self, plan_id: int, plan: dict) -> None:

        self.request_complete_event = threading.Event()
        self.completed_tasks = 0
        self.total_tasks = plan['parallel']

        start_time = time.perf_counter_ns()

        # start rpc tasks
        self.perf_data = []
        for ii in range(plan['parallel']):
            publish_executor = self.executor_vec[ii]
            publish_executor.Execute(lambda: self.StartBenchPlan(plan))

        # wait for all tasks to complete
        self.request_complete_event.wait()

        end_time = time.perf_counter_ns()
        total_time_ms = (end_time - start_time) / 1e6

        self.perf_data.sort()

        correct_count = len(self.perf_data)
        total_count = plan['parallel'] * plan['msg_count']
        error_rate = (total_count - correct_count) / total_count * 100.0
        qps = (total_count * 1000.0) / total_time_ms

        min_latency = self.perf_data[0]
        max_latency = self.perf_data[-1]
        avg_latency = sum(self.perf_data) / correct_count
        p90_latency = self.perf_data[int(correct_count * 0.9)]
        p99_latency = self.perf_data[int(correct_count * 0.99)]
        p999_latency = self.perf_data[int(correct_count * 0.999)]

        result_str = f"Benchmark plan {plan_id} completed, report:"
        if plan['perf_mode'] == 'fixed-freq':
            result_str += f"\nfreq: {plan['freq']}"
        result_str += f"\nmode: {plan['perf_mode']}"
        result_str += f"\nmsg size: {plan['msg_size']}"
        result_str += f"\nparallel: {plan['parallel']}"
        result_str += f"\nmsg count per co: {plan['msg_count']}"
        result_str += f"\ntotal count: {total_count}"
        result_str += f"\ntotal time: {total_time_ms:.2f} ms"
        result_str += f"\ncorrect count: {correct_count}"
        result_str += f"\nerror rate: {error_rate:.2f} %"
        if plan['perf_mode'] == 'bench':
            result_str += f"\nqps: {qps:.2f}"
        result_str += f"\nmin latency: {min_latency:.2f} us"
        result_str += f"\nmax latency: {max_latency:.2f} us"
        result_str += f"\navg latency: {avg_latency:.2f} us"
        result_str += f"\np90 latency: {p90_latency:.2f} us"
        result_str += f"\np99 latency: {p99_latency:.2f} us"
        result_str += f"\np999 latency: {p999_latency:.2f} us\n"
        aimrt_py.info(self.logger, result_str)

    def StartBenchPlan(self, plan: dict) -> None:
        req = rpc_pb2.GetFooDataReq()
        req.msg = self.GenerateRandomString(plan['msg_size'])

        for _ in range(plan['msg_count']):
            ctx = aimrt_py.RpcContext()
            ctx.SetTimeout(datetime.timedelta(seconds=3))
            task_start_time = time.perf_counter_ns()
            status, _ = self.proxy.GetFooData(ctx, req)
            task_end_time = time.perf_counter_ns()

            if status.Code() != aimrt_py.RpcStatusRetCode.OK:
                aimrt_py.error(self.logger, f"GetFooData failed: {status.ToString()}")
                continue
            if task_end_time <= task_start_time:
                aimrt_py.error(self.logger, f"Task end time {task_end_time} is less than start time {task_start_time}")
                continue

            self.perf_data.append((task_end_time - task_start_time) / 1e3)  # us

            if plan['perf_mode'] == 'fixed-freq':
                time.sleep(1 / plan['freq'])

        with threading.Lock():
            self.completed_tasks += 1
            if self.completed_tasks == self.total_tasks:
                self.request_complete_event.set()

    @staticmethod
    def GenerateRandomString(length: int) -> str:
        return ''.join(random.choices(string.ascii_letters + string.digits, k=length))
