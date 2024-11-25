// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "pybind11/pybind11.h"

#include <cstring>
#include <utility>

#include "aimrt_module_c_interface/util/type_support_base.h"
#include "aimrt_module_cpp_interface/util/string.h"

namespace aimrt::runtime::python_runtime {

class PyPbTypeSupport {
 public:
  using BufType = std::string;

  PyPbTypeSupport() : base_(GenBase(this)) {}

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
  void* Create() const {
    return new BufType();
  }

  void Destroy(void* msg) const {
    delete static_cast<BufType*>(msg);
  }

  void Copy(const void* from, void* to) const {
    const auto& from_buf = *static_cast<const BufType*>(from);
    auto& to_buf = *static_cast<BufType*>(to);
    to_buf = from_buf;
  }

  void Move(void* from, void* to) const {
    auto& from_buf = *static_cast<BufType*>(from);
    auto& to_buf = *static_cast<BufType*>(to);
    to_buf = std::move(from_buf);
  }

  bool Serialize(
      aimrt_string_view_t serialization_type,
      const void* msg,
      const aimrt_buffer_array_allocator_t* allocator,
      aimrt_buffer_array_t* buffer_array) const {
    const auto& msg_buf = *static_cast<const BufType*>(msg);

    auto buffer = allocator->allocate(allocator->impl, buffer_array, msg_buf.size());
    if (buffer.data == nullptr || buffer.len < msg_buf.size()) return false;
    memcpy(buffer.data, msg_buf.data(), msg_buf.size());

    return true;
  }

  bool Deserialize(
      aimrt_string_view_t serialization_type,
      aimrt_buffer_array_view_t buffer_array_view,
      void* msg) const {
    auto& msg_buf = *static_cast<BufType*>(msg);

    size_t total_size = 0;
    for (size_t ii = 0; ii < buffer_array_view.len; ++ii) {
      total_size += buffer_array_view.data[ii].len;
    }

    msg_buf.resize(total_size);

    size_t cur_size = 0;
    for (size_t ii = 0; ii < buffer_array_view.len; ++ii) {
      memcpy(msg_buf.data() + cur_size, buffer_array_view.data[ii].data,
             buffer_array_view.data[ii].len);
      cur_size += buffer_array_view.data[ii].len;
    }

    return true;
  }

  static aimrt_type_support_base_t GenBase(void* impl) {
    return aimrt_type_support_base_t{
        .type_name = [](void* impl) -> aimrt_string_view_t {
          return aimrt::util::ToAimRTStringView(static_cast<PyPbTypeSupport*>(impl)->type_name_);
        },
        .create = [](void* impl) -> void* {
          return static_cast<PyPbTypeSupport*>(impl)->Create();
        },
        .destroy = [](void* impl, void* msg) {
          static_cast<PyPbTypeSupport*>(impl)->Destroy(msg);  //
        },
        .copy = [](void* impl, const void* from, void* to) {
          static_cast<PyPbTypeSupport*>(impl)->Copy(from, to);  //
        },
        .move = [](void* impl, void* from, void* to) {
          static_cast<PyPbTypeSupport*>(impl)->Move(from, to);  //
        },
        .serialize = [](void* impl, aimrt_string_view_t serialization_type, const void* msg, const aimrt_buffer_array_allocator_t* allocator, aimrt_buffer_array_t* buffer_array) -> bool {
          return static_cast<PyPbTypeSupport*>(impl)->Serialize(serialization_type, msg, allocator, buffer_array);
        },
        .deserialize = [](void* impl, aimrt_string_view_t serialization_type, aimrt_buffer_array_view_t buffer_array_view, void* msg) -> bool {
          return static_cast<PyPbTypeSupport*>(impl)->Deserialize(serialization_type, buffer_array_view, msg);
        },
        .serialization_types_supported_num = [](void* impl) -> size_t {
          return static_cast<PyPbTypeSupport*>(impl)->serialization_types_supported_list_inner_.size();
        },
        .serialization_types_supported_list = [](void* impl) -> const aimrt_string_view_t* {
          return static_cast<PyPbTypeSupport*>(impl)->serialization_types_supported_list_inner_.data();
        },
        .custom_type_support_ptr = [](void* impl) -> const void* {
          return nullptr;
        },
        .impl = impl};
  }

 private:
  aimrt_type_support_base_t base_;
  std::string type_name_;
  std::vector<std::string> serialization_types_supported_list_;
  std::vector<aimrt_string_view_t> serialization_types_supported_list_inner_;
};

inline void ExportPbTypeSupport(pybind11::object m) {
  pybind11::class_<PyPbTypeSupport, std::shared_ptr<PyPbTypeSupport>>(std::move(m), "PyPbTypeSupport")
      .def(pybind11::init<>())
      .def("SetTypeName", &PyPbTypeSupport::SetTypeName)
      .def("SetSerializationTypesSupportedList", &PyPbTypeSupport::SetSerializationTypesSupportedList);
}

}  // namespace aimrt::runtime::python_runtime
