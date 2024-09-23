// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "parameter_module/parameter_module.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::examples::cpp::parameter::parameter_module {

bool ParameterModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;

  try {
    // Get executor handle
    work_executor_ = core_.GetExecutorManager().GetExecutor("work_thread_pool");
    AIMRT_CHECK_ERROR_THROW(
        work_executor_ && work_executor_.SupportTimerSchedule(),
        "Get executor 'work_thread_pool' failed.");

    parameter_handle_ = core_.GetParameterHandle();
    AIMRT_CHECK_ERROR_THROW(parameter_handle_, "Get parameter failed.");

  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool ParameterModule::Start() {
  try {
    work_executor_.Execute(std::bind(&ParameterModule::SetParameterLoop, this));
    work_executor_.Execute(std::bind(&ParameterModule::GetParameterLoop, this));
  } catch (const std::exception& e) {
    AIMRT_ERROR("Start failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Start succeeded.");
  return true;
}

void ParameterModule::Shutdown() {
  try {
    run_flag_ = false;
    set_loop_stop_sig_.get_future().wait();
    get_loop_stop_sig_.get_future().wait();
  } catch (const std::exception& e) {
    AIMRT_ERROR("Shutdown failed, {}", e.what());
    return;
  }

  AIMRT_INFO("Shutdown succeeded.");
}

void ParameterModule::SetParameterLoop() {
  try {
    AIMRT_INFO("Start SetParameterLoop.");

    uint32_t count = 0;
    while (run_flag_) {
      count++;
      AIMRT_INFO("SetParameterLoop count : {} -------------------------", count);

      std::string key = "key-" + std::to_string(count);
      std::string val = "val-" + std::to_string(count);
      parameter_handle_.SetParameter(key, val);
      AIMRT_INFO("Set parameter, key: '{}', val: '{}'", key, val);

      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    AIMRT_INFO("Exit SetParameterLoop.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("Exit SetParameterLoop with exception, {}", e.what());
  }

  set_loop_stop_sig_.set_value();
}

void ParameterModule::GetParameterLoop() {
  try {
    AIMRT_INFO("Start GetParameterLoop.");

    uint32_t count = 0;
    while (run_flag_) {
      count++;
      AIMRT_INFO("GetParameterLoop count : {} -------------------------", count);

      std::string key = "key-" + std::to_string(count);
      auto val = parameter_handle_.GetParameter(key);
      AIMRT_INFO("Get parameter, key: '{}', val: '{}'", key, val);

      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    AIMRT_INFO("Exit GetParameterLoop.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("Exit GetParameterLoop with exception, {}", e.what());
  }

  get_loop_stop_sig_.set_value();
}

}  // namespace aimrt::examples::cpp::parameter::parameter_module
