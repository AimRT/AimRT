// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string>
#include <unordered_map>

#include "util/string_util.h"

#include <boost/asio.hpp>

#include "net_plugin/global.h"

namespace aimrt::plugins::net_plugin {

template <typename EndPointType>
class MsgHandleRegistry {
 public:
  using MsgHandleFunc =
      std::function<void(const std::shared_ptr<boost::asio::streambuf>&)>;
  using ServerMsgHandleFunc =
      std::function<void(const EndPointType&, const std::shared_ptr<boost::asio::streambuf>&)>;

  MsgHandleRegistry() = default;
  ~MsgHandleRegistry() = default;

  MsgHandleRegistry(const MsgHandleRegistry&) = delete;
  MsgHandleRegistry& operator=(const MsgHandleRegistry&) = delete;

  ServerMsgHandleFunc GetMsgHandleFunc() const {
    return [this](const auto& ep, const auto& buf) { HandleServerMsg(ep, buf); };
  }

  template <typename... Args>
    requires std::constructible_from<MsgHandleFunc, Args...>
  void RegisterMsgHandle(std::string_view uri, Args&&... args) {
    msg_handle_map_.emplace(uri, std::forward<Args>(args)...);
  }

  void Shutdown() {
    if (std::atomic_exchange(&shutdown_flag_, true)) return;
  }

 private:
  void HandleServerMsg(const EndPointType& ep,
                       const std::shared_ptr<boost::asio::streambuf>& buf) const {
    if (shutdown_flag_.load()) [[unlikely]]
      return;

    // 1 byte uri len : n byte uri : buf.len-1-n byte data
    try {
      const void* buf_data = buf->data().data();
      size_t buf_size = buf->size();

      AIMRT_CHECK_ERROR_THROW(buf_size > 1, "Invalid msg, buf size: {}", buf_size);

      uint8_t uri_size = static_cast<const uint8_t*>(buf_data)[0];
      AIMRT_CHECK_ERROR_THROW(buf_size > static_cast<size_t>(uri_size) + 1,
                              "Invalid msg, buf size: {}, uri size: {}",
                              buf_size, uri_size);

      std::string_view uri = std::string_view(static_cast<const char*>(buf_data) + 1, uri_size);
      auto finditr = msg_handle_map_.find(uri);
      if (finditr == msg_handle_map_.end()) [[unlikely]] {
        AIMRT_WARN("Unregisted uri: {}", uri);
        return;
      }

      finditr->second(buf);

    } catch (const std::exception& e) {
      AIMRT_ERROR("Handle msg failed, {}", e.what());
      return;
    }
  }

 private:
  std::atomic_bool shutdown_flag_ = false;

  std::unordered_map<std::string, MsgHandleFunc, aimrt::common::util::StringHash, std::equal_to<>>
      msg_handle_map_;
};

}  // namespace aimrt::plugins::net_plugin
