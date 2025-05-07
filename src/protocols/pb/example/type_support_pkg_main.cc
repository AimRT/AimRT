// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "aimrt_type_support_pkg_c_interface/type_support_pkg_main.h"

#include "aimrt_module_protobuf_interface/util/protobuf_type_support.h"

#include "benchmark.pb.h"
#include "common.pb.h"
#include "event.pb.h"
#include "rpc.pb.h"

static const aimrt_type_support_base_t* type_support_array[]{
    aimrt::GetProtobufMessageTypeSupport<aimrt::protocols::example::BenchmarkSignal>(),
    aimrt::GetProtobufMessageTypeSupport<aimrt::protocols::example::BenchmarkMessage>(),
    aimrt::GetProtobufMessageTypeSupport<aimrt::protocols::example::ExampleFoo>(),
    aimrt::GetProtobufMessageTypeSupport<aimrt::protocols::example::ExampleBar>(),
    aimrt::GetProtobufMessageTypeSupport<aimrt::protocols::example::ExampleEventMsg>(),
};

extern "C" {

size_t AimRTDynlibGetTypeSupportArrayLength() {
  return sizeof(type_support_array) / sizeof(type_support_array[0]);
}

const aimrt_type_support_base_t** AimRTDynlibGetTypeSupportArray() {
  return type_support_array;
}
}