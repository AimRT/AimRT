// Copyright (c) 2024, SmartCar Project
// Iceoryx2 Plugin Test

#include <gtest/gtest.h>
#include <chrono>
#include <cstring>
#include <future>
#include <iostream>
#include <thread>
#include <vector>

#include "aimrt_module_c_interface/util/type_support_base.h"
#include "aimrt_module_cpp_interface/util/type_support.h"
#include "iceoryx2_plugin/iceoryx2_channel_backend.h"

using namespace aimrt::plugins::iceoryx2_plugin;

// --- Mock Type Support for std::string ---

namespace mock {

aimrt_string_view_t TypeName(void*) {
  static const char* name = "std::string";
  return aimrt_string_view_t{name, strlen(name)};
}

void* Create(void*) {
  return new std::string();
}

void Destroy(void*, void* msg) {
  delete static_cast<std::string*>(msg);
}

void Copy(void*, const void* from, void* to) {
  *static_cast<std::string*>(to) = *static_cast<const std::string*>(from);
}

void Move(void*, void* from, void* to) {
  *static_cast<std::string*>(to) = std::move(*static_cast<std::string*>(from));
}

bool Serialize(void*, aimrt_string_view_t, const void* msg,
               const aimrt_buffer_array_allocator_t* allocator,
               aimrt_buffer_array_t* buffer_array) {
  const std::string* str = static_cast<const std::string*>(msg);

  // Allocate buffer using provided allocator
  aimrt_buffer_t buf = allocator->allocate(allocator->impl, buffer_array, str->size());
  if (buf.data) {
    std::memcpy(buf.data, str->data(), str->size());
    return true;
  }
  return false;
}

bool Deserialize(void*, aimrt_string_view_t, aimrt_buffer_array_view_t view, void* msg) {
  std::string* str = static_cast<std::string*>(msg);
  str->clear();
  for (size_t i = 0; i < view.len; ++i) {
    // Correctly access aimrt_buffer_array_view_t -> data (array of buffer_view_t)
    const aimrt_buffer_view_t& buf_view = view.data[i];
    if (buf_view.data && buf_view.len > 0) {
      str->append(static_cast<const char*>(buf_view.data), buf_view.len);
    }
  }
  return true;
}

size_t SerializationTypesSupportedNum(void*) { return 1; }

const aimrt_string_view_t* SerializationTypesSupportedList(void*) {
  static aimrt_string_view_t types[] = {{"default", 7}};
  return types;
}

const void* CustomTypeSupportPtr(void*) { return nullptr; }

static aimrt_type_support_base_t kStringSupportBase = {
    TypeName, Create, Destroy, Copy, Move, Serialize, Deserialize,
    SerializationTypesSupportedNum, SerializationTypesSupportedList, CustomTypeSupportPtr, nullptr};

}  // namespace mock

// --- Test Case ---

TEST(Iceoryx2PluginTest, InitStartShutdown) {
  Iceoryx2ChannelBackend backend;
  YAML::Node options;
  options["listener_thread_name"] = "test_listener";

  EXPECT_NO_THROW(backend.Initialize(options));
  EXPECT_NO_THROW(backend.Start());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_NO_THROW(backend.Shutdown());
}

TEST(Iceoryx2PluginTest, PubSub) {
  Iceoryx2ChannelBackend backend;
  YAML::Node options;
  options["listener_thread_name"] = "pubsub_listener";
  options["use_event_mode"] = false;  // Use polling mode for stable unit tests
  backend.Initialize(options);

  // Mock Type Info
  aimrt::util::TypeSupportRef type_ref(&mock::kStringSupportBase);
  aimrt::runtime::core::channel::TopicInfo topic_info;
  topic_info.msg_type = "std::string";
  topic_info.topic_name = "test_topic";
  topic_info.pkg_path = "test_pkg";
  topic_info.module_name = "test_module";
  topic_info.msg_type_support_ref = type_ref;

  // 1. Register Subscriber
  std::promise<std::string> msg_promise;
  auto msg_future = msg_promise.get_future();

  aimrt::runtime::core::channel::SubscribeWrapper sub_wrapper;
  sub_wrapper.info = topic_info;
  sub_wrapper.callback = [&](aimrt::runtime::core::channel::MsgWrapper& msg_wrapper, std::function<void()>&&) {
    if (msg_wrapper.msg_ptr) {
      msg_promise.set_value(*static_cast<const std::string*>(msg_wrapper.msg_ptr));
    } else {
      msg_promise.set_exception(std::make_exception_ptr(std::runtime_error("Received null msg")));
    }
  };

  EXPECT_TRUE(backend.Subscribe(sub_wrapper));

  // 2. Register Publisher
  aimrt::runtime::core::channel::PublishTypeWrapper pub_wrapper;
  pub_wrapper.info = topic_info;
  EXPECT_TRUE(backend.RegisterPublishType(pub_wrapper));

  backend.Start();

  // 3. Publish Message
  std::string send_msg = "Hello Iceoryx2";
  aimrt::runtime::core::channel::MsgWrapper msg_to_send{topic_info};
  msg_to_send.msg_ptr = &send_msg;

  backend.Publish(msg_to_send);

  // 4. Wait for reception
  auto status = msg_future.wait_for(std::chrono::seconds(2));
  EXPECT_EQ(status, std::future_status::ready);

  if (status == std::future_status::ready) {
    EXPECT_EQ(msg_future.get(), send_msg);
  }

  backend.Shutdown();
}
