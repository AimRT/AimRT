// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <csignal>
#include <iostream>

#include "core/aimrt_core.h"

#include "aimrt_module_cpp_interface/core.h"
#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"

#include "event.pb.h"

using namespace aimrt::runtime::core;

bool run_flag = true;

void SignalHandler(int sig) {
  if (sig == SIGINT || sig == SIGTERM) {
    run_flag = false;
    return;
  }

  raise(sig);
};

int32_t main(int32_t argc, char** argv) {
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);

  std::cout << "AimRT start." << std::endl;

  try {
    AimRTCore core;

    // Initialize
    AimRTCore::Options options;
    if (argc > 1) options.cfg_file_path = argv[1];

    core.Initialize(options);

    // Create Module
    aimrt::CoreRef module_handle(
        core.GetModuleManager().CreateModule("NormalPublisherModule"));

    std::string topic_name = "test_topic";
    double channel_frq = 0.5;

    // Read cfg
    auto file_path = module_handle.GetConfigurator().GetConfigFilePath();
    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(file_path));
      topic_name = cfg_node["topic_name"].as<std::string>();
      channel_frq = cfg_node["channel_frq"].as<double>();
    }

    // Register publish type
    auto publisher = module_handle.GetChannelHandle().GetPublisher(topic_name);
    AIMRT_HL_CHECK_ERROR_THROW(module_handle.GetLogger(),
                               publisher, "Get publisher for topic '{}' failed.", topic_name);

    aimrt::channel::PublisherProxy<aimrt::protocols::example::ExampleEventMsg> publisher_proxy(publisher);
    bool ret = publisher_proxy.RegisterPublishType();
    AIMRT_HL_CHECK_ERROR_THROW(module_handle.GetLogger(), ret, "Register publish type failed.");

    // Start
    auto fu = core.AsyncStart();

    // Publish event
    uint32_t count = 0;
    while (run_flag) {
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<uint32_t>(1000 / channel_frq)));

      count++;
      AIMRT_HL_INFO(module_handle.GetLogger(), "Loop count : {} -------------------------", count);

      // publish event
      aimrt::protocols::example::ExampleEventMsg msg;
      msg.set_msg("count: " + std::to_string(count));
      msg.set_num(count);

      AIMRT_HL_INFO(module_handle.GetLogger(), "Publish new pb event, data: {}", aimrt::Pb2CompactJson(msg));
      publisher_proxy.Publish(msg);
    }

    // Shutdown
    core.Shutdown();

    // Wait
    fu.wait();

  } catch (const std::exception& e) {
    std::cout << "AimRT run with exception and exit. " << e.what()
              << std::endl;
    return -1;
  }

  std::cout << "AimRT exit." << std::endl;

  return 0;
}
