// Copyright (c) 2024, SmartCar Project
// Iceoryx2 Plugin for AimRT - Type Definitions

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>

namespace aimrt::plugins::iceoryx2_plugin {

// Message handler callback type
using MsgHandler = std::function<void(const void* data, size_t size)>;

// Statistics for a single publisher/subscriber
struct Iox2Stats {
  std::atomic<uint64_t> count{0};
  std::atomic<uint64_t> bytes{0};

  Iox2Stats() = default;
  ~Iox2Stats() = default;

  // Non-copyable, non-movable (atomics)
  Iox2Stats(const Iox2Stats&) = delete;
  Iox2Stats& operator=(const Iox2Stats&) = delete;
  Iox2Stats(Iox2Stats&&) = delete;
  Iox2Stats& operator=(Iox2Stats&&) = delete;

  void Add(size_t size) {
    count.fetch_add(1, std::memory_order_relaxed);
    bytes.fetch_add(size, std::memory_order_relaxed);
  }

  uint64_t GetCount() const { return count.load(std::memory_order_relaxed); }
  uint64_t GetBytes() const { return bytes.load(std::memory_order_relaxed); }
};

// Error codes for Iceoryx2 operations
enum class Iox2Error {
  kSuccess = 0,
  kNodeCreationFailed,
  kServiceCreationFailed,
  kPublisherCreationFailed,
  kSubscriberCreationFailed,
  kLoanFailed,
  kSendFailed,
  kReceiveFailed,
  kNotInitialized,
  kEventServiceCreationFailed,
  kNotifierCreationFailed,
  kListenerCreationFailed,
  kWaitSetCreationFailed,
  kWaitSetAttachFailed,
};

inline const char* Iox2ErrorToString(Iox2Error err) {
  switch (err) {
    case Iox2Error::kSuccess:
      return "Success";
    case Iox2Error::kNodeCreationFailed:
      return "Node creation failed";
    case Iox2Error::kServiceCreationFailed:
      return "Service creation failed";
    case Iox2Error::kPublisherCreationFailed:
      return "Publisher creation failed";
    case Iox2Error::kSubscriberCreationFailed:
      return "Subscriber creation failed";
    case Iox2Error::kLoanFailed:
      return "Loan failed";
    case Iox2Error::kSendFailed:
      return "Send failed";
    case Iox2Error::kReceiveFailed:
      return "Receive failed";
    case Iox2Error::kNotInitialized:
      return "Not initialized";
    case Iox2Error::kEventServiceCreationFailed:
      return "Event service creation failed";
    case Iox2Error::kNotifierCreationFailed:
      return "Notifier creation failed";
    case Iox2Error::kListenerCreationFailed:
      return "Listener creation failed";
    case Iox2Error::kWaitSetCreationFailed:
      return "WaitSet creation failed";
    case Iox2Error::kWaitSetAttachFailed:
      return "WaitSet attach failed";
    default:
      return "Unknown error";
  }
}

}  // namespace aimrt::plugins::iceoryx2_plugin
