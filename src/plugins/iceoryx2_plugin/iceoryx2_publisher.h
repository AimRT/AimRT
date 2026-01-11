// Copyright (c) 2024, SmartCar Project
// Iceoryx2 Plugin for AimRT - Publisher Wrapper

#pragma once

#include <cstring>
#include <mutex>
#include <optional>
#include <string>

#include "iceoryx2_plugin/iceoryx2_types.h"
#include "iox2/iceoryx2.hpp"

namespace aimrt::plugins::iceoryx2_plugin {

class Iox2Publisher {
 public:
  using InnerPublisher = iox2::Publisher<iox2::ServiceType::Ipc, iox2::bb::Slice<uint8_t>, void>;
  using InnerNotifier = iox2::Notifier<iox2::ServiceType::Ipc>;

  Iox2Publisher(InnerPublisher&& publisher, std::string url)
      : inner_(std::move(publisher)), url_(std::move(url)) {}

  ~Iox2Publisher() = default;

  // Non-copyable, non-movable (has mutex and stats with atomics)
  Iox2Publisher(const Iox2Publisher&) = delete;
  Iox2Publisher& operator=(const Iox2Publisher&) = delete;
  Iox2Publisher(Iox2Publisher&&) = delete;
  Iox2Publisher& operator=(Iox2Publisher&&) = delete;

  // Set notifier for event-based notification
  void SetNotifier(InnerNotifier&& notifier) {
    notifier_.emplace(std::move(notifier));
  }

  bool HasNotifier() const { return notifier_.has_value(); }

  // Publish data to shared memory (zero-copy write)
  Iox2Error Publish(const void* data, size_t size) {
    std::lock_guard<std::mutex> lock(mtx_);

    auto sample_res = inner_.loan_slice_uninit(size);
    if (!sample_res.has_value()) {
      return Iox2Error::kLoanFailed;
    }

    auto sample_uninit = std::move(sample_res.value());
    auto payload_mut = sample_uninit.payload_mut();
    std::memcpy(payload_mut.data(), data, size);

    auto sample = iox2::assume_init(std::move(sample_uninit));
    auto send_res = iox2::send(std::move(sample));
    if (!send_res.has_value()) {
      return Iox2Error::kSendFailed;
    }

    // Notify subscribers if notifier is set
    if (notifier_) {
      (void)notifier_->notify();  // Ignore result - notification is best-effort
    }

    stats_.Add(size);
    return Iox2Error::kSuccess;
  }

  // Publish from fragmented buffers (for serialized data)
  Iox2Error PublishFragmented(const void* const* fragments, const size_t* sizes, size_t count) {
    size_t total_size = 0;
    for (size_t i = 0; i < count; ++i) {
      total_size += sizes[i];
    }

    std::lock_guard<std::mutex> lock(mtx_);

    auto sample_res = inner_.loan_slice_uninit(total_size);
    if (!sample_res.has_value()) {
      return Iox2Error::kLoanFailed;
    }

    auto sample_uninit = std::move(sample_res.value());
    auto payload_mut = sample_uninit.payload_mut();
    uint8_t* dest = payload_mut.data();

    size_t offset = 0;
    for (size_t i = 0; i < count; ++i) {
      std::memcpy(dest + offset, fragments[i], sizes[i]);
      offset += sizes[i];
    }

    auto sample = iox2::assume_init(std::move(sample_uninit));
    auto send_res = iox2::send(std::move(sample));
    if (!send_res.has_value()) {
      return Iox2Error::kSendFailed;
    }

    // Notify subscribers if notifier is set
    if (notifier_) {
      (void)notifier_->notify();  // Ignore result - notification is best-effort
    }

    stats_.Add(total_size);
    return Iox2Error::kSuccess;
  }

  const std::string& Url() const { return url_; }
  const Iox2Stats& Stats() const { return stats_; }

 private:
  std::mutex mtx_;
  InnerPublisher inner_;
  std::optional<InnerNotifier> notifier_;
  std::string url_;
  Iox2Stats stats_;
};

}  // namespace aimrt::plugins::iceoryx2_plugin
