// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "aimrt_type_support_pkg_c_interface/type_support_pkg_main.h"

#include "aimrt_module_protobuf_interface/util/protobuf_type_support.h"

#include "error.pb.h"
#include "header.pb.h"
#include "module.pb.h"

static const aimrt_type_support_base_t* type_support_array[]{
    aimrt::GetProtobufMessageTypeSupport<aimrt::protocols::common::Error>(),
    aimrt::GetProtobufMessageTypeSupport<aimrt::protocols::common::Header>(),
    aimrt::GetProtobufMessageTypeSupport<aimrt::protocols::common::ModuleStatus>(),
    aimrt::GetProtobufMessageTypeSupport<aimrt::protocols::common::ModuleException>(),
    aimrt::GetProtobufMessageTypeSupport<aimrt::protocols::common::ModuleStatus::Status>(),
};

extern "C" {

size_t AimRTDynlibGetTypeSupportArrayLength() {
  return sizeof(type_support_array) / sizeof(type_support_array[0]);
}

const aimrt_type_support_base_t** AimRTDynlibGetTypeSupportArray() {
  return type_support_array;
}
}