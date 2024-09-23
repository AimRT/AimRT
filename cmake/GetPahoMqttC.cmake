# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "get paho_mqtt_c ...")

set(paho_mqtt_c_DOWNLOAD_URL
    "https://github.com/eclipse/paho.mqtt.c/archive/v1.3.13.tar.gz"
    CACHE STRING "")

if(paho_mqtt_c_LOCAL_SOURCE)
  FetchContent_Declare(
    paho_mqtt_c
    SOURCE_DIR ${paho_mqtt_c_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    paho_mqtt_c
    URL ${paho_mqtt_c_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

FetchContent_GetProperties(paho_mqtt_c)
if(NOT paho_mqtt_c_POPULATED)
  set(PAHO_ENABLE_TESTING
      OFF
      CACHE BOOL "")

  set(PAHO_ENABLE_CPACK
      OFF
      CACHE BOOL "")

  set(PAHO_WITH_SSL
      ON
      CACHE BOOL "")

  set(PAHO_BUILD_SHARED
      OFF
      CACHE BOOL "")

  set(PAHO_BUILD_STATIC
      ON
      CACHE BOOL "")

  FetchContent_MakeAvailable(paho_mqtt_c)

  if(TARGET paho-mqtt3c)
    add_library(paho_mqtt_c::paho-mqtt3c ALIAS paho-mqtt3c)
  endif()

  if(TARGET paho-mqtt3a)
    add_library(paho_mqtt_c::paho-mqtt3a ALIAS paho-mqtt3a)
  endif()

  if(TARGET paho-mqtt3c-static)
    add_library(paho_mqtt_c::paho-mqtt3c-static ALIAS paho-mqtt3c-static)
  endif()

  if(TARGET paho-mqtt3a-static)
    add_library(paho_mqtt_c::paho-mqtt3a-static ALIAS paho-mqtt3a-static)
  endif()

  if(TARGET paho-mqtt3cs)
    add_library(paho_mqtt_c::paho-mqtt3cs ALIAS paho-mqtt3cs)
  endif()

  if(TARGET paho-mqtt3as)
    add_library(paho_mqtt_c::paho-mqtt3as ALIAS paho-mqtt3as)
  endif()

  if(TARGET paho-mqtt3cs-static)
    add_library(paho_mqtt_c::paho-mqtt3cs-static ALIAS paho-mqtt3cs-static)
  endif()

  if(TARGET paho-mqtt3as-static)
    add_library(paho_mqtt_c::paho-mqtt3as-static ALIAS paho-mqtt3as-static)
  endif()

endif()

# import targets:
# paho_mqtt_c::paho-mqtt3c
# paho_mqtt_c::paho-mqtt3a
# paho_mqtt_c::paho-mqtt3c-static
# paho_mqtt_c::paho-mqtt3a-static
# paho_mqtt_c::paho-mqtt3cs
# paho_mqtt_c::paho-mqtt3as
# paho_mqtt_c::paho-mqtt3cs-static
# paho_mqtt_c::paho-mqtt3as-static
