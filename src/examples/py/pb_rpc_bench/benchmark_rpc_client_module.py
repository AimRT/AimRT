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

            # benchmark
            for ii, bench_plan in enumerate(self.bench_plans):
                if not self.run_flag:
                    break

                print(f"Start bench plan {ii}")
                self.StartSinglePlan(ii, bench_plan)
                print(f"End bench plan {ii}")

                time.sleep(self.bench_interval)

            aimrt_py.info(self.logger, "Bench completed.")

        except Exception as e:
            aimrt_py.error(self.logger, f"Exit MainLoop with exception: {e}")

        self.stop_sig.set()

    def StartSinglePlan(self, plan_id: int, plan: dict) -> None:


        self.request_complete_event = threading.Event()
        self.completed_tasks = 0
        self.total_tasks = plan['parallel']

        # start rpc tasks
        self.perf_data = []
        for ii in range(plan['parallel']):
            publish_executor = self.executor_vec[ii]
            publish_executor.Execute(lambda: self.StartBenchPlan(plan))

        # wait for all tasks to complete
        self.request_complete_event.wait()

        self.perf_data.sort()

    def StartBenchPlan(self, plan: dict) -> None:
        req = rpc_pb2.GetFooDataReq()
        req.msg = self.GenerateRandomString(plan['msg_size'])

        for _ in range(plan['msg_count']):
            ctx = aimrt_py.RpcContext()
            task_start_time = time.time()
            status, _ = self.proxy.GetFooData(ctx, req)
            task_end_time = time.time()

            # assert status.ToString() == "OK", f"GetFooData failed: {status}"
            print(f"GetFooData done, status: {status.ToString()}")
            assert task_end_time > task_start_time, "Task end time is less than start time"
            self.perf_data.append(task_end_time - task_start_time)

            if plan['perf_mode'] == 'fixed-freq':
                time.sleep(1 / plan['freq'])

        with threading.Lock():
            self.completed_tasks += 1
            if self.completed_tasks == self.total_tasks:
                self.request_complete_event.set()

    @staticmethod
    def GenerateRandomString(length: int) -> str:
        return ''.join(random.choices(string.ascii_letters + string.digits, k=length))
