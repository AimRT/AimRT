// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "util/string_util.h"

#include "iceoryx_posh/popo/listener.hpp"
#include "iceoryx_posh/popo/untyped_publisher.hpp"
#include "iceoryx_posh/popo/untyped_subscriber.hpp"

namespace aimrt::plugins::iceoryx_plugin {

class IoxLoanedShm {
  friend class IoxPublisher;

 public:
  ~IoxLoanedShm() {
    if (release_func_) release_func_(*this);
  }

  IoxLoanedShm(const IoxLoanedShm&) = delete;
  IoxLoanedShm& operator=(const IoxLoanedShm&) = delete;

  void* Ptr() const { return ptr_; }
  size_t Size() const { return size_; }

 private:
  IoxLoanedShm(void* ptr, size_t size, std::function<void(IoxLoanedShm&)> release_func)
      : ptr_(ptr), size_(size), release_func_(release_func) {}

 private:
  void* ptr_;
  size_t size_;
  std::function<void(IoxLoanedShm&)> release_func_;
};

class IoxPublisher {
 public:
  IoxPublisher(std::string_view url, size_t shm_size);
  ~IoxPublisher() = default;

  IoxPublisher(const IoxPublisher&) = delete;
  IoxPublisher& operator=(const IoxPublisher&) = delete;

  IoxLoanedShm LoanShm(size_t min_size);
  void UpdateLoanShm(IoxLoanedShm& loaned_shm, size_t min_size);
  void PublishShm(IoxLoanedShm& loaned_shm);

 private:
  std::mutex mtx_;
  size_t shm_size_;
  iox::popo::UntypedPublisher publisher_;
};

class IceoryxManager {
 public:
  using MsgHandleFunc = std::function<void(iox::popo::UntypedSubscriber* subscriber)>;

 public:
  IceoryxManager() = default;
  ~IceoryxManager() = default;

  IceoryxManager(const IceoryxManager&) = delete;
  IceoryxManager& operator=(const IceoryxManager&) = delete;

  void Initialize(uint64_t shm_init_size);
  void Shutdown();

  void RegisterPublisher(std::string_view url);
  void RegisterSubscriber(std::string_view url, MsgHandleFunc&& handle);

  IoxPublisher* GetPublisher(std::string_view url);

 private:
  uint64_t shm_init_size_;

  std::unique_ptr<iox::popo::Listener> iox_listener_ptr_;
  std::vector<std::unique_ptr<MsgHandleFunc>> msg_handle_vec_;

  std::unordered_map<
      std::string, std::unique_ptr<IoxPublisher>,
      aimrt::common::util::StringHash, std::equal_to<>>
      iox_pub_registry_;
  std::unordered_map<
      std::string, std::unique_ptr<iox::popo::UntypedSubscriber>>
      iox_sub_registry_;
};

}  // namespace aimrt::plugins::iceoryx_plugin
