// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_cpp_interface/module_base.h"
#include "iceoryx_posh/popo/listener.hpp"
#include "iceoryx_posh/popo/untyped_publisher.hpp"
#include "iceoryx_posh/popo/untyped_subscriber.hpp"
#include "iceoryx_posh/popo/wait_set.hpp"
#include "util/string_util.h"
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
  using WaitSet = iox::popo::WaitSet<>;

 public:
  IceoryxManager() = default;
  ~IceoryxManager() = default;

  IceoryxManager(const IceoryxManager&) = delete;
  IceoryxManager& operator=(const IceoryxManager&) = delete;

  void Initialize(uint64_t shm_init_size);
  void Shutdown();

  void RegisterPublisher(std::string_view url);
  void RegisterSubscriber(std::string_view url, std::string_view executor_name, MsgHandleFunc&& handle);

  void RegisterGetExecutorFunc(
      const std::function<aimrt::executor::ExecutorRef(std::string_view)>& get_executor_func) {
    get_executor_func_ = get_executor_func;
  }

  void StartExecutors();

  IoxPublisher* GetPublisher(std::string_view url);

 private:
  struct IceoryxWaitSetWrapper {
    std::unique_ptr<WaitSet> waitset_ptr;
    std::unique_ptr<aimrt::executor::ExecutorRef> executor_ptr;
  };

 private:
  uint64_t shm_init_size_;
  bool running_flag_ = true;

  aimrt::executor::ExecutorRef executor_;
  std::function<executor::ExecutorRef(std::string_view)> get_executor_func_;
  std::vector<std::shared_ptr<std::promise<void>>> executor_promises_;
  std::vector<std::future<void>> executor_futures_;

  std::unordered_map<
      std::string, std::unique_ptr<IoxPublisher>,
      aimrt::common::util::StringHash, std::equal_to<>>
      iox_pub_registry_;

  std::unordered_map<
      std::unique_ptr<iox::popo::UntypedSubscriber>,
      std::unique_ptr<MsgHandleFunc>>
      iox_sub_registry_;

  std::unordered_map<
      std::string, std::unique_ptr<IceoryxWaitSetWrapper>,
      aimrt::common::util::StringHash, std::equal_to<>>
      iox_sub_waitset_registry_;
};

}  // namespace aimrt::plugins::iceoryx_plugin
