# Copyright (c) 2024 The AimRT Authors.
# AimRT is licensed under Mulan PSL v2.

import time
from dataclasses import dataclass, field
from typing import Dict, List

import aimrt_py
import benchmark_pb2
import yaml
from google.protobuf.json_format import MessageToJson


@dataclass
class MsgRecord:
    recv: bool = False
    send_timestamp: int = 0
    recv_timestamp: int = 0


@dataclass
class TopicRecord:
    topic_name: str
    msg_record_vec: List[MsgRecord] = field(default_factory=list)


class BenchmarkSubscriber(aimrt_py.ModuleBase):
    def __init__(self):
        super().__init__()
        self.core = aimrt_py.CoreRef()
        self.logger = aimrt_py.LoggerRef()
        self.max_topic_number = 0

        self.cur_bench_plan = benchmark_pb2.BenchmarkSignal
        self.topic_record_map: Dict[str, TopicRecord] = {}

    def Info(self) -> aimrt_py.ModuleInfo:
        info = aimrt_py.ModuleInfo()
        info.name = "BenchmarkSubscriberModule"
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

            aimrt_py.info(self.logger, f"Module config: max_topic_number={self.max_topic_number}")

            # signal subscriber
            signal_topic = "benchmark_signal"
            self.signal_subscriber = self.core.GetChannelHandle().GetSubscriber(signal_topic)
            if not self.signal_subscriber:
                raise RuntimeError(f"Get subscriber for topic '{signal_topic}' failed.")

            # set signal and message callbacks
            aimrt_py.Subscribe(self.signal_subscriber, benchmark_pb2.BenchmarkSignal, self.SignalCallback)

            # message subscribers
            for ii in range(self.max_topic_number):
                topic_name = f"test_topic_{ii}"
                subscriber = self.core.GetChannelHandle().GetSubscriber(topic_name)
                if not subscriber:
                    raise RuntimeError(f"Get subscriber for topic '{topic_name}' failed.")
                aimrt_py.Subscribe(subscriber,
                                   benchmark_pb2.BenchmarkMessage,
                                   lambda msg, topic_index=ii: self.MessageCallback(topic_index, msg))

        except Exception as e:
            aimrt_py.error(self.logger, f"Initialize failed: {e}")
            return False

        aimrt_py.info(self.logger, "Init succeeded")
        return True

    def Start(self) -> bool:
        return True

    def Shutdown(self) -> bool:
        try:
            aimrt_py.info(self.logger, "Module is shutting down...")
        except Exception as e:
            aimrt_py.error(self.logger, f"Shutdown failed: {e}")
            return False

        aimrt_py.info(self.logger, "Shutdown succeeded")
        return True

    def SignalCallback(self, signal_msg: benchmark_pb2.BenchmarkSignal) -> None:
        aimrt_py.info(self.logger, f"Received signal: {MessageToJson(signal_msg)}")

        match signal_msg.status:
            case benchmark_pb2.BenchmarkStatus.WarmUp:
                pass

            case benchmark_pb2.BenchmarkStatus.Begin:
                self.cur_bench_plan = signal_msg
                if self.cur_bench_plan.topic_number > self.max_topic_number:
                    raise RuntimeError(f"Topic number in bench plan is larger than max topic number: "
                                       f"{self.cur_bench_plan.topic_number} > {self.max_topic_number}")

                for ii in range(self.cur_bench_plan.topic_number):
                    topic_name = f"test_topic_{ii}"
                    self.topic_record_map[topic_name] = TopicRecord(topic_name=topic_name)
                    self.topic_record_map[topic_name].msg_record_vec = [MsgRecord()
                                                                        for _ in range(self.cur_bench_plan.send_num)]

            case benchmark_pb2.BenchmarkStatus.End:
                self.Evaluate()

            case _:
                aimrt_py.error(self.logger, f"Unknown signal status: {signal_msg.status}")

    def MessageCallback(self, topic_index: int, benchmark_msg: benchmark_pb2.BenchmarkMessage) -> None:
        recv_timestamp = time.time_ns()

        topic_name = f"test_topic_{topic_index}"
        self.topic_record_map[topic_name].msg_record_vec[benchmark_msg.seq].recv = True
        self.topic_record_map[topic_name].msg_record_vec[benchmark_msg.seq].recv_timestamp = recv_timestamp
        self.topic_record_map[topic_name].msg_record_vec[benchmark_msg.seq].send_timestamp = benchmark_msg.timestamp

    def Evaluate(self) -> None:
        # calculate metrics
        latency_vec = []
        for ii in range(self.cur_bench_plan.topic_number):
            topic_name = f"test_topic_{ii}"
            topic_record = self.topic_record_map[topic_name]
            for jj in range(self.cur_bench_plan.send_num):
                msg_record = topic_record.msg_record_vec[jj]
                if not msg_record.recv:
                    aimrt_py.error(self.logger, f"topic '{topic_name}' message seq '{jj}' not received")
                    continue

                if msg_record.recv_timestamp < msg_record.send_timestamp:
                    aimrt_py.error(self.logger, f"topic '{topic_name}' message seq '{jj}' "
                                   "recv timestamp is smaller than send timestamp")
                    continue

                latency_vec.append((msg_record.recv_timestamp - msg_record.send_timestamp) / 1e3)  # us

        latency_vec.sort()
        recv_count = len(latency_vec)

        min_latency = latency_vec[0]
        max_latency = latency_vec[-1]
        p90_latency = latency_vec[int(recv_count * 0.9)]
        p99_latency = latency_vec[int(recv_count * 0.99)]
        p999_latency = latency_vec[int(recv_count * 0.999)]

        send_count = self.cur_bench_plan.send_num * self.cur_bench_plan.topic_number
        loss_rate = (send_count - recv_count) / send_count * 100
        avg_latency = sum(latency_vec) / recv_count

        result_str = f"Benchmark plan {self.cur_bench_plan.bench_plan_id} completed, report:"
        result_str += f"\nfrequency: {self.cur_bench_plan.send_frequency} hz"
        result_str += f"\ntopic number: {self.cur_bench_plan.topic_number}"
        result_str += f"\nmsg size: {self.cur_bench_plan.message_size} bytes"
        result_str += f"\nmsg count per topic: {self.cur_bench_plan.send_num}"
        result_str += f"\nsend count: {send_count}"
        result_str += f"\nrecv count: {recv_count}"
        result_str += f"\nloss rate: {loss_rate:.2f}"
        result_str += f"\nmin latency: {min_latency:.3f} us"
        result_str += f"\nmax latency: {max_latency:.3f} us"
        result_str += f"\navg latency: {avg_latency:.3f} us"
        result_str += f"\np90 latency: {p90_latency:.3f} us"
        result_str += f"\np99 latency: {p99_latency:.3f} us"
        result_str += f"\np999 latency: {p999_latency:.3f} us\n"

        aimrt_py.info(self.logger, result_str)
