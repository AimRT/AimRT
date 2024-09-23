// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "logger_bench_module/logger_bench_module.h"

#include "util/string_util.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::examples::cpp::logger::logger_bench_module {

bool LoggerBenchModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;

  // Read cfg
  auto file_path = core_.GetConfigurator().GetConfigFilePath();
  if (!file_path.empty()) {
    YAML::Node cfg_node = YAML::LoadFile(std::string(file_path));
    log_data_size_vec_ = cfg_node["log_data_size"].as<std::vector<uint32_t>>();
    log_bench_num_ = cfg_node["log_bench_num"].as<uint32_t>();
  }

  auto work_executor = core_.GetExecutorManager().GetExecutor("work_executor");
  AIMRT_CHECK_ERROR_THROW(work_executor, "Get executor 'work_thread_pool' failed.");

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool LoggerBenchModule::Start() {
  AIMRT_INFO("Start succeeded.");

  auto work_executor = core_.GetExecutorManager().GetExecutor("work_executor");

  work_executor.Execute([this, work_executor]() {
    using namespace std::chrono;

    std::vector<std::vector<std::string>> result_table =
        {{"log num", "log data size", "total duration(us)", "avg latency(ns)"}};

    size_t bench_num = log_data_size_vec_.size();

    for (size_t bench_count = 0; bench_count < bench_num; ++bench_count) {
      uint32_t log_data_size = log_data_size_vec_[bench_count];

      // create log data
      std::string log_data;
      log_data.reserve(log_data_size);
      for (size_t ii = 0; ii < log_data_size; ++ii) {
        log_data += std::to_string(ii % 10);
      }

      // bench
      auto start_time_point = steady_clock::now();

      for (size_t ii = 0; ii < log_bench_num_; ++ii) {
        AIMRT_INFO("{}", log_data);
      }

      auto end_time_point = steady_clock::now();

      auto total_duration = end_time_point - start_time_point;

      result_table.emplace_back(
          std::vector<std::string>{
              std::to_string(log_bench_num_),
              std::to_string(log_data_size),
              std::to_string(duration_cast<nanoseconds>(total_duration).count()),
              std::to_string(duration_cast<nanoseconds>(total_duration).count() / log_bench_num_)});

      std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    AIMRT_INFO("report:\n{}", aimrt::common::util::DrawTable(result_table));

    log_loop_stop_sig_.set_value();
  });

  return true;
}

void LoggerBenchModule::Shutdown() {
  log_loop_stop_sig_.get_future().wait();

  AIMRT_INFO("Shutdown succeeded.");
}

}  // namespace aimrt::examples::cpp::logger::logger_bench_module
