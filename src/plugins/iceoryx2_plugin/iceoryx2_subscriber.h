// Copyright (c) 2024, SmartCar Project
// Iceoryx2 Plugin for AimRT - Subscriber Wrapper

#pragma once

#include <optional>
#include <stdexcept>
#include <string>

#include "iceoryx2_plugin/iceoryx2_types.h"
#include "iox2/iceoryx2.hpp"

namespace aimrt::plugins::iceoryx2_plugin {

// Maximum events to consume per poll to prevent starvation
inline constexpr size_t kMaxEventsPerPoll = 256;

// Subscriber wrapper that optionally includes a Listener for event-based notification.
// When Listener is set, this class implements FileDescriptorBased for WaitSet attachment.
class Iox2Subscriber : public iox2::FileDescriptorBased {
 public:
  using InnerSubscriber = iox2::Subscriber<iox2::ServiceType::Ipc, iox2::bb::Slice<uint8_t>, void>;
  using InnerListener = iox2::Listener<iox2::ServiceType::Ipc>;

  Iox2Subscriber(InnerSubscriber&& subscriber, std::string url)
      : inner_(std::move(subscriber)), url_(std::move(url)) {}

  ~Iox2Subscriber() override = default;

  // Non-copyable, non-movable (has stats with atomics)
  Iox2Subscriber(const Iox2Subscriber&) = delete;
  Iox2Subscriber& operator=(const Iox2Subscriber&) = delete;
  Iox2Subscriber(Iox2Subscriber&&) = delete;
  Iox2Subscriber& operator=(Iox2Subscriber&&) = delete;

  // Set listener for event-based notification (enables WaitSet attachment)
  void SetListener(InnerListener&& listener) {
    listener_.emplace(std::move(listener));
  }

  bool HasListener() const { return listener_.has_value(); }

  // FileDescriptorBased interface - returns listener's file descriptor for WaitSet
  // PRECONDITION: HasListener() must be true, otherwise throws std::logic_error
  auto file_descriptor() const -> iox2::FileDescriptorView override {
    // Runtime check (works in both debug and release builds)
    if (!listener_.has_value()) {
      throw std::logic_error("file_descriptor() called without listener set");
    }
    return listener_->file_descriptor();
  }

  void SetHandler(MsgHandler handler) { handler_ = std::move(handler); }

  // Consume any pending events from listener (call after WaitSet wakeup)
  // Limits consumption to prevent starvation if events keep arriving
  void ConsumeEvents(size_t max_events = kMaxEventsPerPoll) {
    if (listener_) {
      // Drain pending events (up to max_events to prevent starvation)
      for (size_t i = 0; i < max_events && listener_->try_wait_one().has_value(); ++i) {
        // Just consume, actual messages are handled by TryReceiveOne
      }
    }
  }

  // Try to receive one message (non-blocking)
  // Returns true if a message was received and processed
  bool TryReceiveOne() {
    auto sample_res = inner_.receive();
    if (!sample_res.has_value()) {
      return false;
    }

    auto sample_opt = std::move(sample_res.value());
    if (!sample_opt.has_value()) {
      return false;  // No new samples
    }

    auto& sample = sample_opt.value();
    auto payload = sample.payload();
    const void* data = payload.data();
    size_t size = payload.number_of_elements();

    if (handler_) {
      handler_(data, size);
    }

    stats_.Add(size);
    return true;
  }

  // Receive all available messages
  size_t ReceiveAll() {
    size_t count = 0;
    while (TryReceiveOne()) {
      ++count;
    }
    return count;
  }

  const std::string& Url() const { return url_; }
  const Iox2Stats& Stats() const { return stats_; }

 private:
  InnerSubscriber inner_;
  std::optional<InnerListener> listener_;
  std::string url_;
  MsgHandler handler_;
  Iox2Stats stats_;
};

}  // namespace aimrt::plugins::iceoryx2_plugin
