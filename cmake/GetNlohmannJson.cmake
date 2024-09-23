# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "get nlohmann_json ...")

set(nlohmann_json_DOWNLOAD_URL
    "https://github.com/nlohmann/json/archive/v3.11.3.tar.gz"
    CACHE STRING "")

if(nlohmann_json_LOCAL_SOURCE)
  FetchContent_Declare(
    nlohmann_json
    SOURCE_DIR ${nlohmann_json_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    nlohmann_json
    URL ${nlohmann_json_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

FetchContent_GetProperties(nlohmann_json)
if(NOT nlohmann_json_POPULATED)
  FetchContent_MakeAvailable(nlohmann_json)
endif()

# import targets:
# nlohmann_json::nlohmann_json
