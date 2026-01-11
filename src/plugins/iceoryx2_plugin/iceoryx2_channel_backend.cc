// Copyright (c) 2024, SmartCar Project
// Iceoryx2 Plugin for AimRT - Channel Backend (Simplified)

#include "iceoryx2_plugin/iceoryx2_channel_backend.h"
#include "iceoryx2_plugin/global.h"

#include "aimrt_module_cpp_interface/channel/channel_context.h"
#include "util/buffer_util.h"
#include "util/url_encode.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::iceoryx2_plugin::Iceoryx2ChannelBackend::Options> {
  using Options = aimrt::plugins::iceoryx2_plugin::Iceoryx2ChannelBackend::Options;

  static Node encode(const Options& rhs) {
    Node node;
    node["shm_init_size"] = rhs.shm_init_size;
    node["max_slice_len"] = rhs.max_slice_len;
    node["allocation_strategy"] = rhs.allocation_strategy;
    node["listener_thread_name"] = rhs.listener_thread_name;
    node["use_event_mode"] = rhs.use_event_mode;
    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;
    if (node["shm_init_size"])
      rhs.shm_init_size = node["shm_init_size"].as<uint64_t>();
    if (node["max_slice_len"])
      rhs.max_slice_len = node["max_slice_len"].as<uint64_t>();
    if (node["allocation_strategy"])
      rhs.allocation_strategy = node["allocation_strategy"].as<std::string>();
    if (node["listener_thread_name"])
      rhs.listener_thread_name = node["listener_thread_name"].as<std::string>();
    if (node["use_event_mode"])
      rhs.use_event_mode = node["use_event_mode"].as<bool>();
    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::iceoryx2_plugin {

Iceoryx2ChannelBackend::~Iceoryx2ChannelBackend() {
  Shutdown();
}

void Iceoryx2ChannelBackend::Initialize(YAML::Node options_node) {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");

  if (options_node && !options_node.IsNull()) {
    options_ = options_node.as<Options>();
  }

  // Validate configuration
  if (options_.max_slice_len > options_.shm_init_size) {
    AIMRT_WARN("max_slice_len ({}) > shm_init_size ({}), this may cause allocation failures",
               options_.max_slice_len, options_.shm_init_size);
  }

  // Validate allocation_strategy
  if (options_.allocation_strategy != "static" &&
      options_.allocation_strategy != "dynamic" &&
      options_.allocation_strategy != "power_of_two") {
    AIMRT_WARN("Unknown allocation_strategy '{}', using 'dynamic'", options_.allocation_strategy);
    options_.allocation_strategy = "dynamic";
  }

  // Create iox2 Node directly - no Manager needed (iox2 has no RouDi)
  auto node_res = iox2::NodeBuilder().create<iox2::ServiceType::Ipc>();
  if (!node_res.has_value()) {
    AIMRT_ERROR_THROW("Failed to create Iceoryx2 Node.");
  }
  node_.emplace(std::move(node_res.value()));

  AIMRT_INFO("Iceoryx2 channel backend initialized.");
}

void Iceoryx2ChannelBackend::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");

  running_ = true;

  // All event mode setup must be done under single lock to prevent race conditions
  // between checking subscribers and attaching to WaitSet
  bool use_event_loop = false;
  {
    std::lock_guard<std::mutex> lock(sub_mtx_);

    bool can_use_event_mode = options_.use_event_mode && !subscribers_.empty();

    // Check if all subscribers have listeners (required for event mode)
    if (can_use_event_mode) {
      for (const auto& [url, sub] : subscribers_) {
        if (!sub->HasListener()) {
          can_use_event_mode = false;
          AIMRT_WARN("Subscriber {} has no listener, falling back to polling mode", url);
          break;
        }
      }
    }

    if (can_use_event_mode) {
      // Create WaitSet and attach all subscribers (still under lock)
      using namespace iox2;
      auto waitset_res = WaitSetBuilder().create<ServiceType::Ipc>();
      if (waitset_res.has_value()) {
        waitset_.emplace(std::move(waitset_res.value()));

        for (const auto& [url, sub] : subscribers_) {
          // Attach with 5 second deadline (will wake up periodically even without events)
          auto guard_res = waitset_->attach_deadline(*sub, bb::Duration::from_secs(5));
          if (guard_res.has_value()) {
            waitset_guards_.push_back(std::move(guard_res.value()));
            waitset_guard_urls_.push_back(url);  // Store URL in same order as guard
            AIMRT_TRACE("Attached subscriber {} to WaitSet", url);
          } else {
            AIMRT_WARN("Failed to attach subscriber {} to WaitSet", url);
          }
        }

        use_event_loop = !waitset_guards_.empty();
        if (use_event_loop) {
          AIMRT_INFO("Iceoryx2 channel backend started in EVENT mode ({} subscribers attached)",
                     waitset_guards_.size());
        }
      } else {
        AIMRT_WARN("Failed to create WaitSet, falling back to polling mode");
      }
    }
  }  // Release lock before starting thread

  // Start appropriate thread (outside lock to avoid holding lock during thread creation)
  if (use_event_loop) {
    poller_thread_ = std::thread([this] { EventLoopFunc(); });
  } else {
    poller_thread_ = std::thread([this] { PollingThreadFunc(); });
    AIMRT_INFO("Iceoryx2 channel backend started in POLLING mode.");
  }
}

void Iceoryx2ChannelBackend::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  running_ = false;
  if (poller_thread_.joinable()) {
    poller_thread_.join();
  }

  // Clear WaitSet guards first (must be released before subscribers)
  waitset_guards_.clear();
  waitset_guard_urls_.clear();
  waitset_.reset();

  {
    std::lock_guard<std::mutex> lock(pub_mtx_);
    publishers_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(sub_mtx_);
    subscribers_.clear();
    subscriber_data_.clear();
  }

  node_.reset();

  AIMRT_INFO("Iceoryx2 channel backend shutdown. Stats: pub={} ({} bytes), sub={} ({} bytes)",
             pub_stats_.GetCount(), pub_stats_.GetBytes(),
             sub_stats_.GetCount(), sub_stats_.GetBytes());
}

bool Iceoryx2ChannelBackend::RegisterPublishType(
    const runtime::core::channel::PublishTypeWrapper& publish_type_wrapper) noexcept {
  try {
    AIMRT_CHECK_ERROR_THROW(state_.load() == State::kInit,
                            "Method can only be called when state is 'Init'.");
    namespace util = aimrt::common::util;

    const auto& info = publish_type_wrapper.info;
    std::string url = std::string("/channel/") +
                      util::UrlEncode(info.topic_name) + "/" +
                      util::UrlEncode(info.msg_type);

    std::lock_guard<std::mutex> lock(pub_mtx_);

    // Check if already registered
    if (publishers_.count(url)) {
      return true;
    }

    // Create publisher via iox2
    using namespace iox2;
    auto service_name_res = ServiceName::create(url.c_str());
    if (!service_name_res.has_value()) {
      AIMRT_ERROR("Invalid service name: {}", url);
      return false;
    }

    auto service_res = node_->service_builder(service_name_res.value())
                           .template publish_subscribe<bb::Slice<uint8_t>>()
                           .open_or_create();
    if (!service_res.has_value()) {
      AIMRT_ERROR("Failed to create service: {}", url);
      return false;
    }

    auto pub_res = service_res.value()
                       .publisher_builder()
                       .initial_max_slice_len(options_.max_slice_len)
                       .create();
    if (!pub_res.has_value()) {
      AIMRT_ERROR("Failed to create publisher: {}", url);
      return false;
    }

    auto publisher = std::make_unique<Iox2Publisher>(std::move(pub_res.value()), url);

    // Create event service and notifier for event-based mode
    if (options_.use_event_mode) {
      auto event_service_res = node_->service_builder(service_name_res.value())
                                   .event()
                                   .open_or_create();
      if (event_service_res.has_value()) {
        auto notifier_res = event_service_res.value().notifier_builder().create();
        if (notifier_res.has_value()) {
          publisher->SetNotifier(std::move(notifier_res.value()));
          AIMRT_TRACE("Created notifier for publisher: {}", url);
        } else {
          AIMRT_WARN("Failed to create notifier for {}, falling back to polling", url);
        }
      } else {
        AIMRT_WARN("Failed to create event service for {}, falling back to polling", url);
      }
    }

    publishers_.emplace(url, std::move(publisher));

    AIMRT_INFO("Register publish type to iceoryx2 channel, url: {}", url);
    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

bool Iceoryx2ChannelBackend::Subscribe(
    const runtime::core::channel::SubscribeWrapper& subscribe_wrapper) noexcept {
  try {
    AIMRT_CHECK_ERROR_THROW(state_.load() == State::kInit,
                            "Method can only be called when state is 'Init'.");

    namespace util = aimrt::common::util;

    const auto& info = subscribe_wrapper.info;
    std::string url = std::string("/channel/") +
                      util::UrlEncode(info.topic_name) + "/" +
                      util::UrlEncode(info.msg_type);

    std::lock_guard<std::mutex> lock(sub_mtx_);

    // Check if already registered
    if (subscribers_.count(url)) {
      return true;
    }

    // Create subscriber via iox2
    using namespace iox2;
    auto service_name_res = ServiceName::create(url.c_str());
    if (!service_name_res.has_value()) {
      AIMRT_ERROR("Invalid service name: {}", url);
      return false;
    }

    auto service_res = node_->service_builder(service_name_res.value())
                           .template publish_subscribe<bb::Slice<uint8_t>>()
                           .open_or_create();
    if (!service_res.has_value()) {
      AIMRT_ERROR("Failed to create service: {}", url);
      return false;
    }

    auto sub_res = service_res.value().subscriber_builder().create();
    if (!sub_res.has_value()) {
      AIMRT_ERROR("Failed to create subscriber: {}", url);
      return false;
    }

    auto subscriber = std::make_unique<Iox2Subscriber>(std::move(sub_res.value()), url);

    // Create event service and listener for event-based mode
    if (options_.use_event_mode) {
      auto event_service_res = node_->service_builder(service_name_res.value())
                                   .event()
                                   .open_or_create();
      if (event_service_res.has_value()) {
        auto listener_res = event_service_res.value().listener_builder().create();
        if (listener_res.has_value()) {
          subscriber->SetListener(std::move(listener_res.value()));
          AIMRT_TRACE("Created listener for subscriber: {}", url);
        } else {
          AIMRT_WARN("Failed to create listener for {}, falling back to polling", url);
        }
      } else {
        AIMRT_WARN("Failed to create event service for subscriber {}, falling back to polling", url);
      }
    }

    // Create SubscribeTool for callback dispatching
    auto tool = std::make_unique<runtime::core::channel::SubscribeTool>();
    tool->AddSubscribeWrapper(&subscribe_wrapper);

    std::string default_serialization_type(info.msg_type_support_ref.DefaultSerializationType());

    // Set handler on subscriber
    //
    // THREAD SAFETY: This handler is called from PollingThreadFunc/EventLoopFunc
    // which always holds sub_mtx_ when calling TryReceiveOne()/ReceiveAll().
    // Do NOT lock sub_mtx_ here to avoid deadlock.
    //
    // The handler safely accesses subscriber_data_ because:
    // 1. subscriber_data_ is only modified in Subscribe() (before Start) and
    //    Shutdown() (after poller thread exits via running_=false + join())
    // 2. Shutdown() sets running_=false BEFORE joining poller thread
    // 3. Poller thread checks running_ before each iteration, ensuring
    //    no new callbacks are started once Shutdown() is initiated
    // 4. join() guarantees all in-flight callbacks complete before
    //    subscriber_data_.clear() is called
    //
    // Buffer format (compatible with iceoryx_plugin):
    //   pkg_size(4) | ser_type_len(1) + ser_type | ctx_num(1) | [ctx_key_len(2) + ctx_key | ctx_val_len(2) + ctx_val]* | msg_buffer
    subscriber->SetHandler([this, url](const void* data, size_t size) {
      namespace util = aimrt::common::util;

      auto it = subscriber_data_.find(url);
      if (it == subscriber_data_.end() || !it->second.tool) {
        return;
      }

      try {
        // Parse header
        util::ConstBufferOperator buf_oper(static_cast<const char*>(data), size);

        uint32_t pkg_size = buf_oper.GetUint32();
        if (pkg_size != size) {
          AIMRT_WARN("Package size mismatch: {} vs {}", pkg_size, size);
        }

        // Get serialization type
        std::string serialization_type(buf_oper.GetString(util::BufferLenType::kUInt8));

        // Create context and set serialization type
        auto ctx_ptr = std::make_shared<aimrt::channel::Context>(
            aimrt_channel_context_type_t::AIMRT_CHANNEL_SUBSCRIBER_CONTEXT);
        ctx_ptr->SetSerializationType(serialization_type);

        // Get context metadata
        size_t ctx_num = buf_oper.GetUint8();
        for (size_t ii = 0; ii < ctx_num; ++ii) {
          auto key = buf_oper.GetString(util::BufferLenType::kUInt16);
          auto val = buf_oper.GetString(util::BufferLenType::kUInt16);
          ctx_ptr->SetMetaValue(key, val);
        }

        ctx_ptr->SetMetaValue(AIMRT_CHANNEL_CONTEXT_KEY_BACKEND, Name());

        // Get remaining buffer (message data)
        auto remaining_buf = buf_oper.GetRemainingBuffer();

        it->second.tool->DoSubscribeCallback(
            ctx_ptr, serialization_type,
            static_cast<const void*>(remaining_buf.data()), remaining_buf.size());

        sub_stats_.Add(size);
      } catch (const std::exception& e) {
        AIMRT_WARN("Failed to parse iceoryx2 message: {}", e.what());
      }
    });

    subscribers_.emplace(url, std::move(subscriber));

    SubscriberData sub_data;
    sub_data.tool = std::move(tool);
    sub_data.default_serialization_type = default_serialization_type;
    subscriber_data_.emplace(url, std::move(sub_data));

    AIMRT_INFO("Subscribe to iceoryx2 channel, url: {}", url);
    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
    return false;
  }
}

void Iceoryx2ChannelBackend::Publish(runtime::core::channel::MsgWrapper& msg_wrapper) noexcept {
  try {
    AIMRT_CHECK_ERROR_THROW(state_.load() == State::kStart,
                            "Method can only be called when state is 'Start'.");

    namespace util = aimrt::common::util;

    const auto& info = msg_wrapper.info;
    std::string url = std::string("/channel/") +
                      util::UrlEncode(info.topic_name) + "/" +
                      util::UrlEncode(info.msg_type);

    // Hold lock for entire publish operation to prevent use-after-free
    // if Shutdown() is called concurrently
    std::lock_guard<std::mutex> lock(pub_mtx_);

    auto it = publishers_.find(url);
    if (it == publishers_.end()) {
      AIMRT_ERROR("Publisher not found for url: {}", url);
      return;
    }
    auto& publisher = it->second;

    // Get serialization type (with null check for ctx_ref)
    std::string_view serialization_type;
    if (msg_wrapper.ctx_ref) {
      serialization_type = msg_wrapper.ctx_ref.GetSerializationType();
    }
    if (serialization_type.empty()) {
      serialization_type = info.msg_type_support_ref.DefaultSerializationType();
    }

    // Get context metadata (with null check for ctx_ref)
    const aimrt_string_view_t* meta_key_vals_array = nullptr;
    size_t meta_key_vals_array_len = 0;
    if (msg_wrapper.ctx_ref) {
      auto [arr, len] = msg_wrapper.ctx_ref.GetMetaKeyValsArray();
      meta_key_vals_array = arr;
      meta_key_vals_array_len = len;
    }
    AIMRT_CHECK_ERROR_THROW(meta_key_vals_array_len / 2 <= 255,
                            "Too much context meta, require less than 255, but actually {}.", meta_key_vals_array_len / 2);

    size_t context_meta_kv_size = 1;  // ctx_num (1 byte)
    for (size_t ii = 0; ii < meta_key_vals_array_len; ++ii) {
      context_meta_kv_size += (2 + meta_key_vals_array[ii].len);  // 2 bytes length + string
    }

    // Serialize message
    auto buffer_view_ptr = runtime::core::channel::TrySerializeMsgWithCache(msg_wrapper, serialization_type);
    if (!buffer_view_ptr) {
      AIMRT_ERROR("Failed to serialize message for topic: {}", url);
      return;
    }

    const auto* data_views = buffer_view_ptr->Data();
    size_t buffer_count = buffer_view_ptr->Size();
    size_t msg_size = buffer_view_ptr->BufferSize();

    // Calculate total SHM size:
    // pkg_size(4) + ser_type_len(1) + ser_type + ctx_meta + msg_buffer
    uint32_t header_size = 4 + 1 + serialization_type.size() + context_meta_kv_size;
    size_t shm_size = header_size + msg_size;

    // OPTIMIZATION: Use thread_local to avoid heap allocation on hot path
    thread_local std::vector<uint8_t> tl_header_buffer;
    thread_local std::vector<const void*> tl_fragments;
    thread_local std::vector<size_t> tl_sizes;

    // Resize (reuses existing capacity if possible)
    tl_header_buffer.resize(header_size);
    tl_fragments.clear();
    tl_sizes.clear();
    tl_fragments.reserve(1 + buffer_count);
    tl_sizes.reserve(1 + buffer_count);

    util::BufferOperator buf_oper(reinterpret_cast<char*>(tl_header_buffer.data()), tl_header_buffer.size());

    // Write pkg_size
    buf_oper.SetUint32(static_cast<uint32_t>(shm_size));

    // Write serialization type
    buf_oper.SetString(serialization_type, util::BufferLenType::kUInt8);

    // Write context metadata
    buf_oper.SetUint8(static_cast<uint8_t>(meta_key_vals_array_len / 2));
    for (size_t ii = 0; ii < meta_key_vals_array_len; ++ii) {
      buf_oper.SetString(aimrt::util::ToStdStringView(meta_key_vals_array[ii]), util::BufferLenType::kUInt16);
    }

    // Build fragment list [Header, Payload_Part1, Payload_Part2, ...]
    tl_fragments.push_back(tl_header_buffer.data());
    tl_sizes.push_back(header_size);

    // Add Payload Fragments
    for (size_t i = 0; i < buffer_count; ++i) {
      tl_fragments.push_back(data_views[i].data);
      tl_sizes.push_back(data_views[i].len);
    }

    // 3. Publish via fragmented API (Zero-copy to SHM)
    auto err = publisher->PublishFragmented(tl_fragments.data(), tl_sizes.data(), tl_fragments.size());
    if (err != Iox2Error::kSuccess) {
      AIMRT_ERROR("Failed to publish to {}: {}", url, Iox2ErrorToString(err));
      return;
    }

    pub_stats_.Add(shm_size);
    AIMRT_TRACE("Iceoryx2 published to '{}' ({} bytes)", url, shm_size);
  } catch (const std::exception& e) {
    AIMRT_ERROR("{}", e.what());
  }
}

void Iceoryx2ChannelBackend::PollingThreadFunc() {
  while (running_) {
    bool has_data = false;

    {
      std::lock_guard<std::mutex> lock(sub_mtx_);
      for (auto& [url, sub] : subscribers_) {
        size_t received = sub->ReceiveAll();
        if (received > 0) {
          has_data = true;
        }
      }
    }

    if (!has_data) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

void Iceoryx2ChannelBackend::EventLoopFunc() {
  using namespace iox2;

  auto on_event = [this](WaitSetAttachmentId<ServiceType::Ipc> id) -> CallbackProgression {
    if (!running_) {
      return CallbackProgression::Stop;
    }

    // Check which subscriber(s) have events and process them
    std::lock_guard<std::mutex> lock(sub_mtx_);

    for (size_t i = 0; i < waitset_guards_.size() && i < waitset_guard_urls_.size(); ++i) {
      if (id.has_event_from(waitset_guards_[i]) || id.has_missed_deadline(waitset_guards_[i])) {
        const auto& url = waitset_guard_urls_[i];
        auto it = subscribers_.find(url);
        if (it != subscribers_.end()) {
          // Consume events from listener first
          it->second->ConsumeEvents();
          // Then receive all available messages
          it->second->ReceiveAll();
        }
      }
    }

    return CallbackProgression::Continue;
  };

  // Run until shutdown - use timeout to periodically check running_ flag
  while (running_) {
    auto result = waitset_->wait_and_process_once_with_timeout(on_event, bb::Duration::from_millis(100));
    if (!result.has_value()) {
      AIMRT_WARN("WaitSet error, switching to polling mode");
      // Actually fall back to polling mode
      PollingThreadFunc();
      return;
    }

    auto run_result = result.value();
    if (run_result == WaitSetRunResult::Interrupt ||
        run_result == WaitSetRunResult::TerminationRequest) {
      AIMRT_INFO("WaitSet received signal, stopping");
      break;
    }
  }
}

}  // namespace aimrt::plugins::iceoryx2_plugin
