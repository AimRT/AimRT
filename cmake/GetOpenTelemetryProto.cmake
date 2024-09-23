# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "get opentelemetry_proto ...")

set(opentelemetry_proto_DOWNLOAD_URL
    "https://github.com/open-telemetry/opentelemetry-proto/archive/v1.3.2.tar.gz"
    CACHE STRING "")

if(opentelemetry_proto_LOCAL_SOURCE)
  FetchContent_Declare(opentelemetry_proto SOURCE_DIR ${opentelemetry_proto_LOCAL_SOURCE})
else()
  FetchContent_Declare(
    opentelemetry_proto
    URL ${opentelemetry_proto_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
endif()

FetchContent_GetProperties(opentelemetry_proto)
if(NOT opentelemetry_proto_POPULATED)
  FetchContent_Populate(opentelemetry_proto)
endif()
