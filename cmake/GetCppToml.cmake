# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "get cpptoml...")

# fetch cpptoml
set(cpptoml_DOWNLOAD_URL
    "https://github.com/skystrife/cpptoml/archive/refs/tags/v0.1.0.tar.gz"
    CACHE STRING "")

if(cpptoml_LOCAL_SOURCE)
  FetchContent_Declare(
    cpptoml
    SOURCE_DIR ${cpptoml_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    cpptoml
    URL ${cpptoml_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP ON
    OVERRIDE_FIND_PACKAGE)
endif()

FetchContent_GetProperties(cpptoml)
if(NOT cpptoml_POPULATED)
  FetchContent_Populate(cpptoml)

  file(READ ${cpptoml_SOURCE_DIR}/include/cpptoml.h CPPTOML_TMP_VAR)
  string(REPLACE "#include <cstring>" "#include <limits>" CPPTOML_TMP_VAR "${CPPTOML_TMP_VAR}")
  file(WRITE ${cpptoml_SOURCE_DIR}/include/cpptoml.h "${CPPTOML_TMP_VAR}")

  file(READ ${cpptoml_SOURCE_DIR}/cmake/cpptomlConfig.cmake.in CPPTOML_TMP_VAR)
  string(REPLACE "\n" ";" CPPTOML_TMP_VAR_LINES "${CPPTOML_TMP_VAR}")
  list(LENGTH CPPTOML_TMP_VAR_LINES CPPTOML_TMP_VAR_LINES_LENGTH)
  if(CPPTOML_TMP_VAR_LINES_LENGTH GREATER 1)
    list(REMOVE_AT CPPTOML_TMP_VAR_LINES 0)
  endif()
  string(REPLACE ";" "\n" CPPTOML_TMP_VAR_LINES "${CPPTOML_TMP_VAR_LINES}")
  file(WRITE ${cpptoml_SOURCE_DIR}/cmake/cpptomlConfig.cmake.in "${CPPTOML_TMP_VAR_LINES}")

  file(READ ${cpptoml_SOURCE_DIR}/CMakeLists.txt CPPTOML_TMP_VAR)
  string(REPLACE " ON" " OFF" CPPTOML_TMP_VAR "${CPPTOML_TMP_VAR}")
  file(WRITE ${cpptoml_SOURCE_DIR}/CMakeLists.txt "${CPPTOML_TMP_VAR}")

  add_subdirectory(${cpptoml_SOURCE_DIR} ${cpptoml_BINARY_DIR})

endif()
