// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "helloworld_module/helloworld_module.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::examples::cpp::helloworld::helloworld_module {

bool HelloWorldModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;

  // Read cfg
  auto file_path = core_.GetConfigurator().GetConfigFilePath();
  if (!file_path.empty()) {
    YAML::Node cfg_node = YAML::LoadFile(std::string(file_path));
    for (const auto& itr : cfg_node) {
      std::string k = itr.first.as<std::string>();
      std::string v = itr.second.as<std::string>();
      AIMRT_INFO("cfg [{} : {}]", k, v);
    }
  }

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool HelloWorldModule::Start() {
  AIMRT_INFO("Start succeeded.");

  return true;
}

void HelloWorldModule::Shutdown() {
  AIMRT_INFO("Shutdown succeeded.");
}

}  // namespace aimrt::examples::cpp::helloworld::helloworld_module
