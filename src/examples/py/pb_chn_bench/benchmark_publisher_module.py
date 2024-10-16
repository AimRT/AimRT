# Copyright (c) 2024 The AimRT Authors.
# AimRT is licensed under Mulan PSL v2.

import os
import random
import signal
import string
import threading
import time

import aimrt_py
import benchmark_pb2
import yaml


class BenchmarkPublisher(aimrt_py.ModuleBase):
    def __init__(self):
        super().__init__()
        self.core = aimrt_py.CoreRef()
        self.logger = aimrt_py.LoggerRef()
        self.max_topic_number = 0
        self.bench_plans = []
        self.run_flag = True
        self.stop_sig = threading.Event()
        self.shutdown_delay = 1  # seconds
        self.bench_interval = 1  # seconds
        self.publish_complete_event = threading.Event()

    def Info(self) -> aimrt_py.ModuleInfo:
        info = aimrt_py.ModuleInfo()
        info.name = "BenchmarkPublisherModule"
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

            self.max_topic_number = cfg_node.get("max_topic_number", 0)

            if "bench_plans" in cfg_node and isinstance(cfg_node["bench_plans"], list):
                for bench_plan_node in cfg_node["bench_plans"]:
                    bench_plan = {
                        "channel_frq": bench_plan_node["channel_frq"],
                        "msg_size": bench_plan_node["msg_size"],
                        "topic_number": bench_plan_node["topic_number"],
                        "msg_count": bench_plan_node["msg_count"]
                    }

                    if bench_plan["topic_number"] > self.max_topic_number:
                        raise ValueError(f"Bench plan topic number ({bench_plan['topic_number']}) "
                                         f"is greater than max topic number ({self.max_topic_number})")

                    self.bench_plans.append(bench_plan)

            aimrt_py.info(self.logger, f"Module config: max_topic_number={self.max_topic_number}, "
                                       f"bench_plans={self.bench_plans}")

            # controller
            self.publish_control_executor = self.core.GetExecutorManager().GetExecutor("publish_control_executor")
            if not self.publish_control_executor:
                raise RuntimeError("Get executor 'publish_control_executor' failed.")

            # signal publisher
            signal_topic = "benchmark_signal"
            self.signal_publisher = self.core.GetChannelHandle().GetPublisher(signal_topic)
            if not self.signal_publisher:
                raise RuntimeError(f"Get publisher for topic '{signal_topic}' failed.")

            ret = aimrt_py.RegisterPublishType(self.signal_publisher, benchmark_pb2.BenchmarkSignal)
            if not ret:
                raise RuntimeError(f"Register publish type for topic '{signal_topic}' failed.")

            # message publishers
            self.publisher_wrapper_vec = []
            for ii in range(self.max_topic_number):
                executor_name = f"publish_executor_{ii}"
                executor = self.core.GetExecutorManager().GetExecutor(executor_name)
                if not executor:
                    raise RuntimeError(f"Get executor '{executor_name}' failed.")

                topic_name = f"test_topic_{ii}"
                publisher = self.core.GetChannelHandle().GetPublisher(topic_name)
                if not publisher:
                    raise RuntimeError(f"Get publisher for topic '{topic_name}' failed.")

                ret = aimrt_py.RegisterPublishType(publisher, benchmark_pb2.BenchmarkMessage)
                if not ret:
                    raise RuntimeError(f"Register publish type for topic '{topic_name}' failed.")

                self.publisher_wrapper_vec.append({
                    "publish_executor": executor,
                    "publisher": publisher
                })

        except Exception as e:
            aimrt_py.error(self.logger, f"Initialize failed: {e}")
            return False

        aimrt_py.info(self.logger, "Init succeeded")
        return True

    def Start(self) -> bool:
        try:
            aimrt_py.info(self.logger, "Module starting ...")
            self.publish_control_executor.Execute(self.MainLoop)

        except Exception as e:
            aimrt_py.error(self.logger, f"Start failed: {e}")
            return False

        aimrt_py.info(self.logger, "Start succeeded")
        return True

    def Shutdown(self) -> bool:
        try:
            aimrt_py.info(self.logger, "Module shutting down ...")

            self.run_flag = False
            self.publish_complete_event.set()
            self.stop_sig.wait()
        except Exception as e:
            aimrt_py.error(self.logger, f"Shutdown failed: {e}")
            return False

        aimrt_py.info(self.logger, "Shutdown succeeded")
        return True

    def MainLoop(self) -> None:
        try:
            aimrt_py.info(self.logger, "Start Bench.")

            # warm up
            for ii in range(10):
                begin_signal = benchmark_pb2.BenchmarkSignal()
                begin_signal.status = benchmark_pb2.BenchmarkStatus.WarmUp
                aimrt_py.Publish(self.signal_publisher, begin_signal)
                time.sleep(0.1)
                aimrt_py.info(self.logger, f"Warm up {ii}")

            # benchmark
            for ii, bench_plan in enumerate(self.bench_plans):
                if not self.run_flag:
                    break

                print(f"Start bench plan {ii}")
                self.StartSinglePlan(ii, bench_plan)
                print(f"End bench plan {ii}")

                time.sleep(self.bench_interval)

            self.ShutdownPeer()

        except Exception as e:
            aimrt_py.error(self.logger, f"Exit MainLoop with exception: {e}")

        self.stop_sig.set()

    def StartSinglePlan(self, plan_id: int, plan: dict) -> None:
        # publish start signal
        begin_signal = benchmark_pb2.BenchmarkSignal()
        begin_signal.status = benchmark_pb2.BenchmarkStatus.Begin
        begin_signal.bench_plan_id = plan_id
        begin_signal.topic_number = plan['topic_number']
        begin_signal.send_num = plan['msg_count']
        begin_signal.message_size = plan['msg_size']
        begin_signal.send_frequency = plan['channel_frq']

        aimrt_py.info(self.logger, f"Publish benchmark start signal, data: \n{begin_signal}")
        aimrt_py.Publish(self.signal_publisher, begin_signal)

        self.publish_complete_event.clear()
        self.completed_tasks = 0
        self.total_tasks = plan['topic_number']

        time.sleep(1)

        # publish topic
        for ii in range(plan['topic_number']):
            publish_executor = self.publisher_wrapper_vec[ii]["publish_executor"]
            publish_executor.Execute(lambda index=ii: self.PublishTask(self.publisher_wrapper_vec[index], plan))

        self.publish_complete_event.wait()

        # publish end signal
        end_signal = benchmark_pb2.BenchmarkSignal()
        end_signal.status = benchmark_pb2.BenchmarkStatus.End
        end_signal.bench_plan_id = plan_id
        aimrt_py.info(self.logger, f"Publish benchmark end signal, data: {end_signal}")
        aimrt_py.Publish(self.signal_publisher, end_signal)

    def PublishTask(self, publisher_wrapper: dict, plan: dict) -> None:
        publisher = publisher_wrapper['publisher']
        msg = benchmark_pb2.BenchmarkMessage()
        msg.data = self.GenerateRandomString(plan['msg_size'])

        send_count = 0
        sleep_time = 1 / plan['channel_frq']

        while send_count < plan['msg_count'] and self.run_flag:
            msg.seq = send_count
            msg.timestamp = int(time.time() * 1e9)  # ns

            aimrt_py.Publish(publisher, msg)

            send_count += 1
            time.sleep(sleep_time)

        with threading.Lock():
            self.completed_tasks += 1
            if self.completed_tasks == self.total_tasks:
                self.publish_complete_event.set()

    def ShutdownPeer(self) -> None:
        shutdown_peer_signal = benchmark_pb2.BenchmarkSignal()
        shutdown_peer_signal.status = benchmark_pb2.BenchmarkStatus.ShutdownPeer
        aimrt_py.info(self.logger, f"Publish benchmark shutdown peer signal, data: {shutdown_peer_signal}")
        aimrt_py.Publish(self.signal_publisher, shutdown_peer_signal)

        time.sleep(self.shutdown_delay)
        os.kill(os.getpid(), signal.SIGINT)

    @staticmethod
    def GenerateRandomString(length: int) -> bytes:
        return ''.join(random.choices(string.ascii_letters + string.digits, k=length)).encode()
