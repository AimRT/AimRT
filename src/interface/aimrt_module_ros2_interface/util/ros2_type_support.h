// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstring>
#include <string>

#include "aimrt_module_c_interface/util/type_support_base.h"
#include "aimrt_module_cpp_interface/util/string.h"
#include "aimrt_module_ros2_interface/util/ros2_rcl_serialized_message_adapter.h"
#include "ros2_util/json_convert.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosidl_runtime_cpp/message_type_support_decl.hpp"
#include "rosidl_runtime_cpp/traits.hpp"
#include "yaml_convert.h"

namespace aimrt {

template <typename T>
concept Ros2MsgType = rosidl_generator_traits::is_message<T>::value;

template <Ros2MsgType MsgType>
const aimrt_type_support_base_t* GetRos2MessageTypeSupport() {
  static const aimrt_string_view_t kChannelRos2SerializationTypesSupportedList[] = {
      aimrt::util::ToAimRTStringView("ros2"), aimrt::util::ToAimRTStringView("json"), aimrt::util::ToAimRTStringView("yaml")};

  static const std::string kMsgTypeName =
      std::string("ros2:") + rosidl_generator_traits::name<MsgType>();

  static const aimrt_type_support_base_t kTs{
      .type_name = [](void* impl) -> aimrt_string_view_t {
        return aimrt::util::ToAimRTStringView(kMsgTypeName);
      },
      .create = [](void* impl) -> void* {
        return new MsgType();
      },
      .destroy = [](void* impl, void* msg) {
        delete static_cast<MsgType*>(msg);  //
      },
      .copy = [](void* impl, const void* from, void* to) {
        *static_cast<MsgType*>(to) = *static_cast<const MsgType*>(from);  //
      },
      .move = [](void* impl, void* from, void* to) {
        *static_cast<MsgType*>(to) = std::move(*static_cast<MsgType*>(from));  //
      },
      .serialize = [](void* impl, aimrt_string_view_t serialization_type, const void* msg, const aimrt_buffer_array_allocator_t* allocator, aimrt_buffer_array_t* buffer_array) -> bool {
        try {
          static const rosidl_message_type_support_t* ts_ptr =
              rosidl_typesupport_cpp::get_message_type_support_handle<MsgType>();

          if (aimrt::util::ToStdStringView(serialization_type) == "ros2") {
            aimrt_buffer_array_with_allocator_t bawa{.buffer_array = buffer_array, .allocator = allocator};
            Ros2RclSerializedMessageAdapter serialized_msg_adapter(&bawa);
            rcl_ret_t ret = rmw_serialize(msg, ts_ptr, serialized_msg_adapter.GetRclSerializedMessage());

            return (ret == RMW_RET_OK);
          }
          if (aimrt::util::ToStdStringView(serialization_type) == "json") {
            std::string msg_data;
            bool ret = common::ros2_util::MessageToJson(msg, ts_ptr, msg_data);
            if (!ret) return false;

            auto buffer = allocator->allocate(allocator->impl, buffer_array, msg_data.size());
            if (buffer.data == nullptr || buffer.len < msg_data.size()) return false;
            memcpy(buffer.data, msg_data.c_str(), msg_data.size());

            return true;
          }
          if (aimrt::util::ToStdStringView(serialization_type) == "yaml") {
            const MsgType* typed_msg = static_cast<const MsgType*>(msg);
            std::string msg_yaml_str = to_yaml(*typed_msg);
            if (msg_yaml_str.empty()) return false;

            if (msg_yaml_str.back() == '\n') {
              msg_yaml_str.pop_back();
            }
            msg_yaml_str.push_back('\0');

            auto buffer = allocator->allocate(allocator->impl, buffer_array, msg_yaml_str.size());
            if (buffer.data == nullptr || buffer.len < msg_yaml_str.size()) return false;
            memcpy(buffer.data, msg_yaml_str.c_str(), msg_yaml_str.size());

            return true;
          }
        } catch (const std::exception& e) {
        }
        return false;
      },
      .deserialize = [](void* impl, aimrt_string_view_t serialization_type, aimrt_buffer_array_view_t buffer_array_view, void* msg) -> bool {
        try {
          static const rosidl_message_type_support_t* ts_ptr =
              rosidl_typesupport_cpp::get_message_type_support_handle<MsgType>();

          if (aimrt::util::ToStdStringView(serialization_type) == "ros2") {
            if (buffer_array_view.len == 1) {
              rcl_serialized_message_t serialized_msg = rmw_get_zero_initialized_serialized_message();
              serialized_msg.buffer = const_cast<uint8_t*>(static_cast<const uint8_t*>(buffer_array_view.data[0].data));
              serialized_msg.buffer_length = buffer_array_view.data[0].len;
              serialized_msg.buffer_capacity = buffer_array_view.data[0].len;
              rcl_ret_t ret = rmw_deserialize(&serialized_msg, ts_ptr, msg);
              return (ret == RMW_RET_OK);
            }

            if (buffer_array_view.len > 1) {
              size_t total_size = 0;
              for (size_t ii = 0; ii < buffer_array_view.len; ++ii) {
                total_size += buffer_array_view.data[ii].len;
              }
              std::vector<uint8_t> buffer_vec(total_size);
              uint8_t* buffer = buffer_vec.data();
              size_t cur_size = 0;
              for (size_t ii = 0; ii < buffer_array_view.len; ++ii) {
                memcpy(buffer + cur_size,
                       buffer_array_view.data[ii].data,
                       buffer_array_view.data[ii].len);
                cur_size += buffer_array_view.data[ii].len;
              }

              rcl_serialized_message_t serialized_msg = rmw_get_zero_initialized_serialized_message();
              serialized_msg.buffer = buffer;
              serialized_msg.buffer_length = total_size;
              serialized_msg.buffer_capacity = total_size;
              rcl_ret_t ret = rmw_deserialize(&serialized_msg, ts_ptr, msg);
              return (ret == RMW_RET_OK);
            }
          } else if (aimrt::util::ToStdStringView(serialization_type) == "json") {
            std::string json_data;
            for (size_t ii = 0; ii < buffer_array_view.len; ++ii) {
              json_data.append((char*)buffer_array_view.data[ii].data, buffer_array_view.data[ii].len);
            }
            bool ret = common::ros2_util::JsonToMessage(json_data, ts_ptr, msg);
            return ret;
          } else if (aimrt::util::ToStdStringView(serialization_type) == "yaml") {
            std::string yaml_data;
            for (size_t ii = 0; ii < buffer_array_view.len; ++ii) {
              yaml_data.append((char*)buffer_array_view.data[ii].data, buffer_array_view.data[ii].len);
            }
            bool ret = common::ros2_util::YamlToMessage(yaml_data, ts_ptr, msg);
            return ret;
          }
        } catch (const std::exception& e) {
        }
        return false;
      },
      .serialization_types_supported_num = [](void* impl) -> size_t {
        return sizeof(kChannelRos2SerializationTypesSupportedList) /
               sizeof(kChannelRos2SerializationTypesSupportedList[0]);
      },
      .serialization_types_supported_list = [](void* impl) -> const aimrt_string_view_t* {
        return kChannelRos2SerializationTypesSupportedList;
      },
      .custom_type_support_ptr = [](void* impl) -> const void* {
        return rosidl_typesupport_cpp::get_message_type_support_handle<MsgType>();
      },
      .impl = nullptr};
  return &kTs;
}

}  // namespace aimrt
