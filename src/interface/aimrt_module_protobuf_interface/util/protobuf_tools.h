// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/util/json_util.h>

namespace aimrt {

inline std::string Pb2PrettyJson(const ::google::protobuf::Message& st) {
  static ::google::protobuf::util::JsonPrintOptions op = []() {
    ::google::protobuf::util::JsonPrintOptions op;
#if GOOGLE_PROTOBUF_VERSION >= 5026000
    op.always_print_fields_with_no_presence = true;
#else
    op.always_print_primitive_fields = true;
#endif
    op.always_print_enums_as_ints = false;
    op.preserve_proto_field_names = true;
    op.add_whitespace = true;
    return op;
  }();

  std::string str;
  std::ignore = ::google::protobuf::util::MessageToJsonString(st, &str, op);
  return str;
}

inline std::string Pb2CompactJson(const ::google::protobuf::Message& st) {
  static ::google::protobuf::util::JsonPrintOptions op = []() {
#if GOOGLE_PROTOBUF_VERSION >= 5026000
    op.always_print_fields_with_no_presence = true;
#else
    op.always_print_primitive_fields = true;
#endif
    op.always_print_enums_as_ints = false;
    op.preserve_proto_field_names = true;
    op.add_whitespace = false;
    return op;
  }();

  std::string str;
  std::ignore = ::google::protobuf::util::MessageToJsonString(st, &str, op);
  return str;
}
}  // namespace aimrt
