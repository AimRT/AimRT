// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <chrono>
#include <optional>

#include "aimrt_module_cpp_interface/executor/executor.h"

#include "tbb/concurrent_hash_map.h"

namespace aimrt::runtime::core::util {

template <typename MsgRecorder>
class RpcClientTool {
 public:
  using TimeoutHandle = std::function<void(MsgRecorder&&)>;

 public:
  RpcClientTool() = default;
  ~RpcClientTool() = default;

  void RegisterTimeoutExecutor(aimrt::executor::ExecutorRef timeout_executor) {
    if (!timeout_executor.SupportTimerSchedule())
      throw std::runtime_error("Timeout executor do not support timer schedule.");

    timeout_executor_ = timeout_executor;
  }

  void RegisterTimeoutHandle(const TimeoutHandle& timeout_handle) {
    timeout_handle_ = timeout_handle;
  }

  bool Record(uint32_t req_id, std::chrono::nanoseconds timeout, MsgRecorder&& msg_recorder) {
    bool ret = client_msg_recorder_map_.emplace(req_id, std::move(msg_recorder));

    if (!ret) [[unlikely]]
      return false;

    if (timeout_executor_) {
      timeout_executor_.ExecuteAfter(timeout, [this, req_id]() {
        auto msg_recorder_op = GetRecord(req_id);
        if (msg_recorder_op) [[unlikely]]
          timeout_handle_(std::move(*msg_recorder_op));
      });
    }

    return true;
  }

  std::optional<MsgRecorder> GetRecord(uint32_t req_id) {
    typename ClientMsgRecorderMap::accessor ac;
    bool find_ret = client_msg_recorder_map_.find(ac, req_id);

    if (!find_ret) [[unlikely]]
      return {};

    auto result = std::make_optional<MsgRecorder>(std::move(ac->second));

    client_msg_recorder_map_.erase(ac);

    return result;
  }

 private:
  aimrt::executor::ExecutorRef timeout_executor_;
  TimeoutHandle timeout_handle_;

  using ClientMsgRecorderMap = tbb::concurrent_hash_map<uint32_t, MsgRecorder>;
  ClientMsgRecorderMap client_msg_recorder_map_;
};

}  // namespace aimrt::runtime::core::util
