// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "iceoryx_plugin/iceoryx_manager.h"
#include "iceoryx_plugin/global.h"

#if defined(_WIN32)
  #include <windows.h>
#else
  #include <unistd.h>
#endif

namespace aimrt::plugins::iceoryx_plugin {

using IdString_t = iox::capro::IdString_t;

// iceoryx rules each part of the service name should be less than 100 characters :IdString_t = cxx::string<100>;
std::string TruncateString(std::string_view input) {
  if (input.length() <= iox::MAX_RUNTIME_NAME_LENGTH) [[likely]] {
    return std::string(input);
  }

  size_t ellipsis_length = 3;
  size_t prefix_length = (iox::MAX_RUNTIME_NAME_LENGTH - ellipsis_length) / 2;
  size_t suffix_length = iox::MAX_RUNTIME_NAME_LENGTH - ellipsis_length - prefix_length;

  std::string truncated_string;
  truncated_string.reserve(iox::MAX_RUNTIME_NAME_LENGTH);
  truncated_string.append(input.substr(0, prefix_length));
  truncated_string.append("...");
  truncated_string.append(input.substr(input.length() - suffix_length));

  AIMRT_WARN("Input url is too long. Each part should be less than {} characters. The input :{} has been truncated to :{}, which may lead to potential risks.",
             iox::MAX_RUNTIME_NAME_LENGTH, input, truncated_string);

  return truncated_string;
}

// iceoryx uses the iox::cxx::TruncateToCapacity_t to limit the length of the string to 100 characters
IdString_t String2IdString(std::string_view str) {
  std::string truncated_str = TruncateString(str);
  iox::cxx::TruncateToCapacity_t truncate_to_capacity;
  return {truncate_to_capacity, truncated_str.c_str(), truncated_str.length()};
}

// The iox description has three parts: "service name", "instance", "specific object"
iox::capro::ServiceDescription Url2ServiceDescription(std::string_view url) {
  size_t first_slash_pos = url.find('/');
  size_t second_slash_pos = url.find('/', first_slash_pos + 1);
  size_t third_slash_pos = url.find('/', second_slash_pos + 1);

  return iox::capro::ServiceDescription{
      String2IdString(url.substr(0, second_slash_pos - 0)),
      String2IdString(url.substr(second_slash_pos, third_slash_pos - second_slash_pos)),
      String2IdString(url.substr(third_slash_pos))};
}

IoxPublisher::IoxPublisher(std::string_view url, size_t shm_size)
    : shm_size_(shm_size),
      publisher_(Url2ServiceDescription(url)) {}

IoxLoanedShm IoxPublisher::LoanShm(size_t min_size) {
  std::lock_guard<std::mutex> lock(mtx_);

  while (min_size > shm_size_) {
    shm_size_ *= 2;
  }

  auto loan_result = publisher_.loan(shm_size_);
  AIMRT_ASSERT(!loan_result.has_error(), "Failed to loan shm");

  return IoxLoanedShm(loan_result.value(), shm_size_, [this](IoxLoanedShm& loaned_shm) {
    if (loaned_shm.ptr_) {
      std::lock_guard<std::mutex> lock(mtx_);
      publisher_.release(loaned_shm.ptr_);
      loaned_shm.ptr_ = nullptr;
    }
  });
}

void IoxPublisher::UpdateLoanShm(IoxLoanedShm& loaned_shm, size_t min_size) {
  std::lock_guard<std::mutex> lock(mtx_);

  while (min_size > shm_size_) {
    shm_size_ *= 2;
  }

  if (loaned_shm.ptr_) {
    publisher_.release(loaned_shm.ptr_);
    loaned_shm.ptr_ = nullptr;
  }

  auto loan_result = publisher_.loan(shm_size_);
  AIMRT_ASSERT(!loan_result.has_error(), "Failed to loan shm");

  loaned_shm.ptr_ = loan_result.value();
  loaned_shm.size_ = shm_size_;
}

void IoxPublisher::PublishShm(IoxLoanedShm& loaned_shm) {
  std::lock_guard<std::mutex> lock(mtx_);

  publisher_.publish(loaned_shm.ptr_);
  loaned_shm.ptr_ = nullptr;
}

void IceoryxManager::Initialize(uint64_t shm_init_size) {
  shm_init_size_ = shm_init_size;

#if defined(_WIN32)
  std::string runtime_id = "iceoryx" + std::to_string(GetProcessId(GetCurrentProcess()));
#else
  std::string runtime_id = "iceoryx" + std::to_string(getpid());
#endif
  iox::runtime::PoshRuntime::initRuntime(iox::RuntimeName_t(iox::cxx::TruncateToCapacity, runtime_id));
}

void IceoryxManager::Shutdown() {
  running_flag_ = false;

  for (auto& [topic_name, waitset_wrapper] : iox_sub_waitset_registry_) {
    if (waitset_wrapper && waitset_wrapper->waitset_ptr) {
      waitset_wrapper->waitset_ptr->markForDestruction();
    }
  }

  iox_sub_registry_.clear();
  iox_pub_registry_.clear();
  iox_sub_waitset_registry_.clear();
}

void IceoryxManager::RegisterPublisher(std::string_view url) {
  iox_pub_registry_.emplace(url, std::make_unique<IoxPublisher>(url, shm_init_size_));
}

void OnReceivedCallback(iox::popo::UntypedSubscriber* subscriber, IceoryxManager::MsgHandleFunc* handle) {
  (*handle)(subscriber);
}

void IceoryxManager::RegisterSubscriber(std::string_view url, std::string_view executor_name, MsgHandleFunc&& handle) {
  auto [it, inserted] = iox_sub_waitset_registry_.try_emplace(
      std::string(executor_name),
      std::make_unique<IceoryxWaitSetWrapper>());

  if (inserted) {
    it->second->waitset_ptr = std::make_unique<WaitSet>();
    it->second->executor_ptr = std::make_unique<aimrt::executor::ExecutorRef>(
        get_executor_func_(executor_name));
  }

  auto subscriber_ptr = std::make_unique<iox::popo::UntypedSubscriber>(Url2ServiceDescription(url));

  it->second->waitset_ptr->attachState(*subscriber_ptr, iox::popo::SubscriberState::HAS_DATA).or_else([](auto) {
    std::cerr << "failed to attach subscriber" << std::endl;
    std::exit(EXIT_FAILURE);
  });

  iox_sub_registry_.emplace(std::move(subscriber_ptr), std::make_unique<MsgHandleFunc>(std::move(handle)));
}

IoxPublisher* IceoryxManager::GetPublisher(std::string_view url) {
  auto it = iox_pub_registry_.find(url);
  if (it != iox_pub_registry_.end())
    return it->second.get();
  return nullptr;
}

void IceoryxManager::StartExecutors() {
  // 创建 subscriber 到 callback 的映射
  std::shared_ptr<std::unordered_map<iox::popo::UntypedSubscriber*, std::function<void(iox::popo::UntypedSubscriber*)>>>
      subscriber_callbacks = std::make_shared<std::unordered_map<iox::popo::UntypedSubscriber*, std::function<void(iox::popo::UntypedSubscriber*)>>>();

  // 填充映射
  for (const auto& pair : iox_sub_registry_) {
    (*subscriber_callbacks)[pair.first.get()] = *(pair.second);
  }

  // 遍历 waitset registry
  for (auto& [topic_name, waitset_wrapper] : iox_sub_waitset_registry_) {
    if (!waitset_wrapper || !waitset_wrapper->executor_ptr || !waitset_wrapper->waitset_ptr) {
      continue;
    }

    // 获取指针的副本，而不是引用
    auto* executor_ptr = waitset_wrapper->executor_ptr.get();
    auto* waitset_raw_ptr = waitset_wrapper->waitset_ptr.get();

    // 使用值捕获而不是引用捕获
    executor_ptr->Execute([this, waitset_raw_ptr, callbacks = subscriber_callbacks]() {
      while (running_flag_) {
        auto notificationVector = waitset_raw_ptr->wait();

        for (auto& notification : notificationVector) {
          auto* subscriber_ptr = notification->getOrigin<iox::popo::UntypedSubscriber>();

          auto callback_it = callbacks->find(subscriber_ptr);
          if (callback_it != callbacks->end()) {
            callback_it->second(subscriber_ptr);
          }
        }
      }
    });
  }
}

}  // namespace aimrt::plugins::iceoryx_plugin