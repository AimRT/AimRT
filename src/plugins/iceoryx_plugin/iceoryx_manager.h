// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once
#include "iceoryx_plugin/util.h"
#include "iceoryx_posh/popo/listener.hpp"
#include "iceoryx_posh/popo/untyped_publisher.hpp"
#include "iceoryx_posh/popo/untyped_subscriber.hpp"

namespace aimrt::plugins::iceoryx_plugin {
using MsgHandleFunc = std::function<void(iox::popo::UntypedSubscriber* subscriber)>;
class IceoryxManager {
 public:
  bool RegisterSubscriber(std::string& url, MsgHandleFunc&& handle);
  bool RegisterPublisher(std::string& url);

  void Initialize();
  void Shutdown();

  auto GetPublisherRegisterMap() const {
    return std::make_shared<std::unordered_map<std::string, std::shared_ptr<IoxPubCtx>>>(iox_pub_registry_);
  }

  struct IoxPubCtx {
    std::shared_ptr<iox::popo::UntypedPublisher> publisher;
    std::mutex mutex;

    explicit IoxPubCtx(std::shared_ptr<iox::popo::UntypedPublisher> pub)
        : publisher(std::move(pub)) {}
  };

 private:
  std::vector<std::shared_ptr<iox::popo::Listener>> iox_listener_vec_;
  std::vector<std::shared_ptr<MsgHandleFunc>> msg_handle_vec_;

  std::unordered_map<std::string, std::shared_ptr<IoxPubCtx>> iox_pub_registry_;
  std::unordered_map<std::string, std::shared_ptr<iox::popo::UntypedSubscriber>> iox_sub_registry_;

  std::mutex registry_mutex_;

  std::atomic<bool> is_initialized_ = false;

  std::string pid_;
};

}  // namespace aimrt::plugins::iceoryx_plugin
