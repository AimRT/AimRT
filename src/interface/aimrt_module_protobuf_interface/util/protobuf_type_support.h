// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstring>
#include <string>
#include <vector>

#include "aimrt_module_c_interface/util/type_support_base.h"
#include "aimrt_module_cpp_interface/context/details/type_support.h"
#include "aimrt_module_cpp_interface/util/string.h"
#include "aimrt_module_protobuf_interface/util/protobuf_zero_copy_stream.h"

#if GOOGLE_PROTOBUF_VERSION >= 3022000
  #include <google/protobuf/json/json.h>
#else
  #include <google/protobuf/stubs/stringpiece.h>
#endif
#include <google/protobuf/message.h>
#include <google/protobuf/util/json_util.h>

namespace aimrt {

template <std::derived_from<::google::protobuf::Message> MsgType>
const aimrt_type_support_base_t* GetProtobufMessageTypeSupport() {
  static const aimrt_string_view_t kChannelProtobufSerializationTypesSupportedList[] = {
      aimrt::util::ToAimRTStringView("pb"),
      aimrt::util::ToAimRTStringView("json")};

  static const std::string kMsgTypeName = "pb:" + MsgType().GetTypeName();

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
          const MsgType& msg_ref = *static_cast<const MsgType*>(msg);
          if (aimrt::util::ToStdStringView(serialization_type) == "pb") {
            BufferArrayZeroCopyOutputStream os(buffer_array, allocator);
            if (!msg_ref.SerializeToZeroCopyStream(&os)) return false;
            os.CommitLastBuf();
            return true;
          }

          if (aimrt::util::ToStdStringView(serialization_type) == "json") {
            // todo: Use zerocopy
            ::google::protobuf::util::JsonPrintOptions op;
#if GOOGLE_PROTOBUF_VERSION >= 5026000
            op.always_print_fields_with_no_presence = true;
#else
            op.always_print_primitive_fields = true;
#endif
            op.preserve_proto_field_names = true;
            std::string str;
            auto status = ::google::protobuf::util::MessageToJsonString(msg_ref, &str, op);
            if (!status.ok()) return false;

            auto buffer = allocator->allocate(allocator->impl, buffer_array, str.size());
            if (buffer.data == nullptr || buffer.len < str.size()) return false;
            memcpy(buffer.data, str.c_str(), str.size());
            return true;
          }

        } catch (...) {
        }
        return false;
      },
      .deserialize = [](void* impl, aimrt_string_view_t serialization_type, aimrt_buffer_array_view_t buffer_array_view, void* msg) -> bool {
        try {
          if (aimrt::util::ToStdStringView(serialization_type) == "pb") {
            BufferArrayZeroCopyInputStream is(buffer_array_view);
            if (!static_cast<MsgType*>(msg)->ParseFromZeroCopyStream(&is))
              return false;
            return true;
          }

          if (aimrt::util::ToStdStringView(serialization_type) == "json") {
            // todo: Use zerocopy
            if (buffer_array_view.len == 1) {
#if GOOGLE_PROTOBUF_VERSION >= 3022000
              auto status = ::google::protobuf::json::JsonStringToMessage(
                  absl::string_view(
                      static_cast<const char*>(buffer_array_view.data[0].data),
                      buffer_array_view.data[0].len),
                  static_cast<MsgType*>(msg),
                  ::google::protobuf::util::JsonParseOptions());
#else
              auto status = ::google::protobuf::util::JsonStringToMessage(
                  ::google::protobuf::StringPiece(
                      static_cast<const char*>(buffer_array_view.data[0].data),
                      buffer_array_view.data[0].len),
                  static_cast<MsgType*>(msg),
                  ::google::protobuf::util::JsonParseOptions());
#endif
              return status.ok();
            }

            if (buffer_array_view.len > 1) {
              size_t total_size = 0;
              for (size_t ii = 0; ii < buffer_array_view.len; ++ii) {
                total_size += buffer_array_view.data[ii].len;
              }
              std::vector<char> buffer_vec(total_size);
              char* buffer = buffer_vec.data();
              size_t cur_size = 0;
              for (size_t ii = 0; ii < buffer_array_view.len; ++ii) {
                memcpy(buffer + cur_size, buffer_array_view.data[ii].data,
                       buffer_array_view.data[ii].len);
                cur_size += buffer_array_view.data[ii].len;
              }
#if GOOGLE_PROTOBUF_VERSION >= 5022000
              auto status = ::google::protobuf::util::JsonStringToMessage(
                  absl::string_view(
                      static_cast<const char*>(buffer), total_size),
                  static_cast<MsgType*>(msg),
                  ::google::protobuf::util::JsonParseOptions());
#else
              auto status = ::google::protobuf::util::JsonStringToMessage(
                  ::google::protobuf::StringPiece(
                      static_cast<const char*>(buffer), total_size),
                  static_cast<MsgType*>(msg),
                  ::google::protobuf::util::JsonParseOptions());
#endif
              return status.ok();
            }
            return false;
          }
        } catch (...) {
        }
        return false;
      },
      .serialization_types_supported_num = [](void* impl) -> size_t {
        return sizeof(kChannelProtobufSerializationTypesSupportedList) /
               sizeof(kChannelProtobufSerializationTypesSupportedList[0]);
      },
      .serialization_types_supported_list = [](void* impl) -> const aimrt_string_view_t* {
        return kChannelProtobufSerializationTypesSupportedList;
      },
      .custom_type_support_ptr = [](void* impl) -> const void* {
        return MsgType::GetDescriptor();
      },
      .impl = nullptr};
  return &kTs;
}

template <std::derived_from<::google::protobuf::Message> MsgType>
struct MessageTypeSupportTraits<MsgType> {
  static const aimrt_type_support_base_t* Get() {
    return GetProtobufMessageTypeSupport<MsgType>();
  }
};

}  // namespace aimrt
