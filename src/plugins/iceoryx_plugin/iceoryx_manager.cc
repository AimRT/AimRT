// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "iceoryx_plugin/iceoryx_manager.h"

namespace aimrt::plugins::iceoryx_plugin {

void OnReceivedCallback(iox::popo::UntypedSubscriber* subscriber, MsgHandleFunc* handle) {
  (*handle)(subscriber);
}
void IceoryxManager::Initialize() {
  pid_ = GetPid();
}

void IceoryxManager::Shutdown() {
  iox_pub_registry_.clear();
  iox_sub_registry_.clear();
  iox_listener_vec_.clear();
  msg_handle_vec_.clear();
}

// The url format is /XX/YY/ZZ, which is expected to be "service name", "instance", "specific Object"
bool IceoryxManager::RegisterPublisher(std::string& url) {
  if (is_initialized_.load(std::memory_order_relaxed) == false) {
    char app_name[iox::MAX_RUNTIME_NAME_LENGTH];
    std::snprintf(app_name, iox::MAX_RUNTIME_NAME_LENGTH, "pub%s", pid_.c_str());

    iox::runtime::PoshRuntime::initRuntime(app_name);

    is_initialized_.store(true, std::memory_order_relaxed);
  }

  try {
    std::shared_ptr<iox::popo::UntypedPublisher> publisher_ptr = std::make_shared<iox::popo::UntypedPublisher>(Url2ServiceDescription(url));

    iox_pub_registry_.emplace(url, std::make_shared<IoxPubCtx>(publisher_ptr));

    return true;

  } catch (const std::exception& e) {
    AIMRT_ERROR("Failed to register subscriber for url: {}, error: {}!", url, e.what());
  }
  return false;
}

bool IceoryxManager::RegisterSubscriber(std::string& url, MsgHandleFunc&& handle) {
  if (is_initialized_.load(std::memory_order_relaxed) == false) {
    char app_name[iox::MAX_RUNTIME_NAME_LENGTH];
    std::snprintf(app_name, iox::MAX_RUNTIME_NAME_LENGTH, "sub%s", pid_.c_str());

    iox::runtime::PoshRuntime::initRuntime(app_name);

    is_initialized_.store(true, std::memory_order_relaxed);
  }

  auto handle_ptr = std::make_shared<MsgHandleFunc>(handle);

  try {
    std::shared_ptr<iox::popo::UntypedSubscriber> subscriber_ptr = std::make_shared<iox::popo::UntypedSubscriber>(Url2ServiceDescription(url));
    auto listener_ptr = std::make_shared<iox::popo::Listener>();
    listener_ptr
        ->attachEvent(*subscriber_ptr,
                      iox::popo::SubscriberEvent::DATA_RECEIVED,
                      iox::popo::createNotificationCallback<iox::popo::UntypedSubscriber, MsgHandleFunc>(OnReceivedCallback, *handle_ptr))
        .or_else([](auto) {
          std::cerr << "Unable to attach a subscriber!" << std::endl;
          std::exit(EXIT_FAILURE);
        });

    iox_listener_vec_.emplace_back(std::move(listener_ptr));
    iox_sub_registry_.emplace(url, std::move(subscriber_ptr));
    msg_handle_vec_.emplace_back(std::move(handle_ptr));

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("Failed to register subscriber for url: {}, error: {}!", url, e.what());
  }
  return false;
}

}  // namespace aimrt::plugins::iceoryx_plugin