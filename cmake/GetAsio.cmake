# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "get asio ...")

set(asio_DOWNLOAD_URL
    "https://github.com/chriskohlhoff/asio/archive/asio-1-30-2.tar.gz"
    CACHE STRING "")

if(asio_LOCAL_SOURCE)
  FetchContent_Declare(
    asio
    SOURCE_DIR ${asio_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    asio
    URL ${asio_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

# Wrap it in a function to restrict the scope of the variables
function(get_asio)
  FetchContent_GetProperties(asio)
  if(NOT asio_POPULATED)
    FetchContent_Populate(asio)

    add_library(asio INTERFACE)
    add_library(asio::asio ALIAS asio)

    target_include_directories(asio INTERFACE $<BUILD_INTERFACE:${asio_SOURCE_DIR}/asio/include> $<INSTALL_INTERFACE:include/asio>)
    target_compile_definitions(asio INTERFACE ASIO_STANDALONE ASIO_NO_DEPRECATED)

    file(GLOB_RECURSE head_files ${asio_SOURCE_DIR}/asio/include/*.hpp ${asio_SOURCE_DIR}/asio/include/*.ipp)
    target_sources(asio INTERFACE FILE_SET HEADERS BASE_DIRS ${asio_SOURCE_DIR}/asio/include FILES ${head_files})

    find_package(Threads REQUIRED)
    target_link_libraries(asio INTERFACE Threads::Threads)

    if(WIN32)
      # macro see @ https://stackoverflow.com/a/40217291/1746503
      macro(get_win32_winnt version)
        if(CMAKE_SYSTEM_VERSION)
          set(ver ${CMAKE_SYSTEM_VERSION})
          string(REGEX MATCH "^([0-9]+).([0-9])" ver ${ver})
          string(REGEX MATCH "^([0-9]+)" verMajor ${ver})
          # Check for Windows 10, b/c we'll need to convert to hex 'A'.
          if("${verMajor}" MATCHES "10")
            set(verMajor "A")
            string(REGEX REPLACE "^([0-9]+)" ${verMajor} ver ${ver})
          endif("${verMajor}" MATCHES "10")
          # Remove all remaining '.' characters.
          string(REPLACE "." "" ver ${ver})
          # Prepend each digit with a zero.
          string(REGEX REPLACE "([0-9A-Z])" "0\\1" ver ${ver})
          set(${version} "0x${ver}")
        endif()
      endmacro()

      if(NOT DEFINED _WIN32_WINNT)
        get_win32_winnt(ver)
        set(_WIN32_WINNT ${ver})
      endif()

      message(STATUS "Set _WIN32_WINNET=${_WIN32_WINNT}")

      target_compile_definitions(asio INTERFACE _WIN32_WINNT=${_WIN32_WINNT} WIN32_LEAN_AND_MEAN)
    endif()

    set_property(TARGET asio PROPERTY EXPORT_NAME asio::asio)
    if(AIMRT_INSTALL)
      install(
        TARGETS asio
        EXPORT asio-config
        FILE_SET HEADERS
        DESTINATION include/asio)

      install(EXPORT asio-config DESTINATION lib/cmake/asio)
    endif()
  endif()
endfunction()

get_asio()

# import targets：
# asio::asio
