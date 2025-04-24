// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <algorithm>
#include <memory>
#include <span>

#include "aimrt_module_c_interface/util/type_support_base.h"
#include "aimrt_module_cpp_interface/util/buffer.h"
#include "aimrt_module_cpp_interface/util/string.h"

namespace aimrt::util {

/**
 * @brief TypeSupportRef
 * @note Basic components, priority is given to performance protection. Please check the legality of base_ptr by yourself
 *
 */
class TypeSupportRef {
 public:
  TypeSupportRef() = default;
  explicit TypeSupportRef(const aimrt_type_support_base_t* base_ptr)
      : base_ptr_(base_ptr) {}
  ~TypeSupportRef() = default;

  explicit operator bool() const { return (base_ptr_ != nullptr); }

  const aimrt_type_support_base_t* NativeHandle() const {
    return base_ptr_;
  }

  std::string_view TypeName() const {
    return ToStdStringView(base_ptr_->type_name(base_ptr_->impl));
  }

  void* Create() const {
    return base_ptr_->create(base_ptr_->impl);
  }

  void Destroy(void* msg) const {
    base_ptr_->destroy(base_ptr_->impl, msg);
  }

  std::shared_ptr<void> CreateSharedPtr() const {
    return std::shared_ptr<void>(
        base_ptr_->create(base_ptr_->impl),
        [base_ptr{this->base_ptr_}](void* ptr) {
          base_ptr->destroy(base_ptr->impl, ptr);
        });
  }

  void Copy(const void* from, void* to) const {
    base_ptr_->copy(base_ptr_->impl, from, to);
  }

  void Move(void* from, void* to) const {
    base_ptr_->move(base_ptr_->impl, from, to);
  }

  bool Serialize(
      std::string_view serialization_type,
      const void* msg,
      const aimrt_buffer_array_allocator_t* allocator,
      aimrt_buffer_array_t* buffer_array) const {
    return base_ptr_->serialize(
        base_ptr_->impl,
        ToAimRTStringView(serialization_type),
        msg,
        allocator,
        buffer_array);
  }

  bool Deserialize(
      std::string_view serialization_type,
      aimrt_buffer_array_view_t buffer_array_view,
      void* msg) const {
    return base_ptr_->deserialize(
        base_ptr_->impl,
        ToAimRTStringView(serialization_type),
        buffer_array_view,
        msg);
  }

  size_t SerializationTypesSupportedNum() const {
    return base_ptr_->serialization_types_supported_num(base_ptr_->impl);
  }

  const aimrt_string_view_t* SerializationTypesSupportedList() const {
    return base_ptr_->serialization_types_supported_list(base_ptr_->impl);
  }

  std::span<const aimrt_string_view_t> SerializationTypesSupportedListSpan() const {
    return std::span<const aimrt_string_view_t>(
        base_ptr_->serialization_types_supported_list(base_ptr_->impl),
        base_ptr_->serialization_types_supported_num(base_ptr_->impl));
  }

  const void* CustomTypeSupportPtr() const {
    return base_ptr_->custom_type_support_ptr(base_ptr_->impl);
  }

  std::string_view DefaultSerializationType() const {
    return ToStdStringView(base_ptr_->serialization_types_supported_list(base_ptr_->impl)[0]);
  }

  bool CheckSerializationTypeSupported(std::string_view serialization_type) const {
    auto serialization_type_span = SerializationTypesSupportedListSpan();
    auto finditr = std::find_if(
        serialization_type_span.begin(), serialization_type_span.end(),
        [serialization_type](aimrt_string_view_t s) {
          return serialization_type == ToStdStringView(s);
        });

    return (finditr != serialization_type_span.end());
  }

  std::string SimpleSerialize(std::string_view serialization_type, const void* msg) const {
    aimrt::util::BufferArray buffer_array;

    bool ret = base_ptr_->serialize(
        base_ptr_->impl,
        ToAimRTStringView(serialization_type),
        msg,
        buffer_array.AllocatorNativeHandle(),
        buffer_array.BufferArrayNativeHandle());

    if (ret)
      return buffer_array.JoinToString();

    return "";
  }

 private:
  const aimrt_type_support_base_t* base_ptr_ = nullptr;
};

}  // namespace aimrt::util
