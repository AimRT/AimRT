// Copyright(c) 2024 The AimRT Authors.
// AimRT is licensed under Mulan PSL v2.

#pragma once

#include "pybind11/pybind11.h"

#include <memory>

#include "aimrt_module_c_interface/util/type_support_base.h"
#include "aimrt_module_cpp_interface/util/string.h"
#include "aimrt_module_ros2_interface/util/ros2_rcl_serialized_message_adapter.h"

#include "pybind11/functional.h"
#include "pybind11/stl.h"
#include "rosidl_runtime_cpp/message_type_support_decl.hpp"

#include "python_runtime/ros2_type_support_utils.h"

namespace aimrt::runtime::python_runtime {

namespace py = pybind11;

class PyRos2TypeSupport {
 public:
  explicit PyRos2TypeSupport(py::object pymsg_type) : base_(GenBase(this, pymsg_type)) {}

  void SetTypeName(const std::string& s) {
    type_name_ = s;
  }

  void SetSerializationTypesSupportedList(const std::vector<std::string>& v) {
    serialization_types_supported_list_ = v;
    serialization_types_supported_list_inner_.clear();
    for (auto& item : serialization_types_supported_list_) {
      serialization_types_supported_list_inner_.emplace_back(aimrt::util::ToAimRTStringView(item));
    }
  }

  const aimrt_type_support_base_t* NativeHandle() const { return &base_; }

 private:
  void* Create() const { return create_ros_message_(); }

  void Destroy(void* msg) const { destroy_ros_message_(msg); }

  void Copy(const void* from, void* to) const {
    const auto* members_info = GetRosMembersInfo(msg_type_support_);
    if (members_info == nullptr) [[unlikely]] {
      throw std::runtime_error("Failed to get message members info.");
    }

    for (size_t ii = 0; ii < members_info->member_count_; ++ii) {
      const auto& member_info = members_info->members_[ii];
      const void* from_ptr = static_cast<const void*>(static_cast<const uint8_t*>(from) + member_info.offset_);
      void* to_ptr = static_cast<void*>(static_cast<uint8_t*>(to) + member_info.offset_);
      if (member_info.is_array_) {
        // TODO(zhangyi1357): Handle array type.
        throw std::runtime_error("Array type is not supported for copy operation.");
      } else {
        CopyBasicMember(member_info, from_ptr, to_ptr);
      }
    }
  }

  void Move(void* from, void* to) const {
    throw std::runtime_error("Move operation is not supported.");
    const auto* members_info = GetRosMembersInfo(msg_type_support_);
    if (members_info == nullptr) [[unlikely]] {
      throw std::runtime_error("Failed to get message members info.");
    }
    for (size_t ii = 0; ii < members_info->member_count_; ++ii) {
      const auto& member_info = members_info->members_[ii];
      const void* from_ptr = static_cast<const void*>(static_cast<const uint8_t*>(from) + member_info.offset_);
      void* to_ptr = static_cast<void*>(static_cast<uint8_t*>(to) + member_info.offset_);
      if (member_info.is_array_) {
        // TODO(zhangyi1357): Handle array type.
        throw std::runtime_error("Array type is not supported for move operation.");
      } else {
        CopyBasicMember(member_info, from_ptr, to_ptr);
      }
    }
  }

  bool Serialize(
      aimrt_string_view_t serialization_type,
      const void* msg,
      const aimrt_buffer_array_allocator_t* allocator,
      aimrt_buffer_array_t* buffer_array) const {
    try {
      if (aimrt::util::ToStdStringView(serialization_type) == "ros2") {
        aimrt_buffer_array_with_allocator_t bawa{.buffer_array = buffer_array, .allocator = allocator};
        Ros2RclSerializedMessageAdapter serialized_msg_adapter(&bawa);
        rcl_ret_t ret = rmw_serialize(msg, msg_type_support_, serialized_msg_adapter.GetRclSerializedMessage());
        return (ret == RMW_RET_OK);
      }
      return false;
    } catch (const std::exception& e) {
      return false;
    }
  }

  bool Deserialize(
      aimrt_string_view_t serialization_type,
      aimrt_buffer_array_view_t buffer_array_view,
      void* msg) const {
    try {
      if (aimrt::util::ToStdStringView(serialization_type) == "ros2") {
        if (buffer_array_view.len == 1) {
          rcl_serialized_message_t serialized_msg = rmw_get_zero_initialized_serialized_message();
          serialized_msg.buffer = const_cast<uint8_t*>(static_cast<const uint8_t*>(buffer_array_view.data[0].data));
          serialized_msg.buffer_length = buffer_array_view.data[0].len;
          serialized_msg.buffer_capacity = buffer_array_view.data[0].len;
          rcl_ret_t ret = rmw_deserialize(&serialized_msg, msg_type_support_, msg);
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
          rcl_ret_t ret = rmw_deserialize(&serialized_msg, msg_type_support_, msg);
          return (ret == RMW_RET_OK);
        }
      }
      return false;
    } catch (const std::exception& e) {
      return false;
    }
  }

  aimrt_type_support_base_t GenBase(PyRos2TypeSupport* impl, py::object pymsg_type) {
    msg_type_support_ = static_cast<rosidl_message_type_support_t*>(common_get_type_support(pymsg_type));

    py::object pymetaclass = pymsg_type.attr("__class__");
    py::object value = pymetaclass.attr("_CREATE_ROS_MESSAGE");
    auto* capsule_ptr = static_cast<void*>(value.cast<py::capsule>());
    create_ros_message_ = reinterpret_cast<create_ros_message_function*>(capsule_ptr);
    // TODO(zhangyi): check create_ros_message_ is not nullptr
    // As this is in the constructor, throw exception might not be a good idea.

    value = pymetaclass.attr("_DESTROY_ROS_MESSAGE");
    capsule_ptr = static_cast<void*>(value.cast<py::capsule>());
    destroy_ros_message_ = reinterpret_cast<destroy_ros_message_function*>(capsule_ptr);
    // TODO(zhangyi): check destroy_ros_message_ is not nullptr
    // As this is in the constructor, throw exception might not be a good idea.

    return aimrt_type_support_base_t{
        .type_name = [](void* impl) -> aimrt_string_view_t {
          return aimrt::util::ToAimRTStringView(static_cast<PyRos2TypeSupport*>(impl)->type_name_);
        },
        .create = [](void* impl) -> void* {
          return static_cast<PyRos2TypeSupport*>(impl)->Create();
        },
        .destroy = [](void* impl, void* msg) {  //
          static_cast<PyRos2TypeSupport*>(impl)->Destroy(msg);
        },
        .copy = [](void* impl, const void* from, void* to) {  //
          static_cast<PyRos2TypeSupport*>(impl)->Copy(from, to);
        },
        .move = [](void* impl, void* from, void* to) {  //
          static_cast<PyRos2TypeSupport*>(impl)->Move(from, to);
        },
        .serialize = [](void* impl, aimrt_string_view_t serialization_type, const void* msg, const aimrt_buffer_array_allocator_t* allocator, aimrt_buffer_array_t* buffer_array) -> bool {
          return static_cast<PyRos2TypeSupport*>(impl)->Serialize(serialization_type, msg, allocator, buffer_array);
        },
        .deserialize = [](void* impl, aimrt_string_view_t serialization_type, aimrt_buffer_array_view_t buffer_array_view, void* msg) -> bool {
          return static_cast<PyRos2TypeSupport*>(impl)->Deserialize(serialization_type, buffer_array_view, msg);
        },
        .serialization_types_supported_num = [](void* impl) -> size_t {
          return static_cast<PyRos2TypeSupport*>(impl)->serialization_types_supported_list_inner_.size();
        },
        .serialization_types_supported_list = [](void* impl) -> const aimrt_string_view_t* {
          return static_cast<PyRos2TypeSupport*>(impl)->serialization_types_supported_list_inner_.data();
        },
        .custom_type_support_ptr = [](void* impl) -> const void* {
          return static_cast<PyRos2TypeSupport*>(impl)->msg_type_support_;
        },
        .impl = impl};
  }

 private:
  rosidl_message_type_support_t* msg_type_support_;
  create_ros_message_function* create_ros_message_;
  destroy_ros_message_function* destroy_ros_message_;

  aimrt_type_support_base_t base_;
  std::string type_name_;
  std::vector<std::string> serialization_types_supported_list_;
  std::vector<aimrt_string_view_t> serialization_types_supported_list_inner_;
};

inline void ExportRos2TypeSupport(py::object m) {
  py::class_<PyRos2TypeSupport, std::shared_ptr<PyRos2TypeSupport>>(std::move(m), "PyRos2TypeSupport")
      .def(py::init<py::object>())
      .def("SetTypeName", &PyRos2TypeSupport::SetTypeName)
      .def("SetSerializationTypesSupportedList", &PyRos2TypeSupport::SetSerializationTypesSupportedList);
}

}  // namespace aimrt::runtime::python_runtime
