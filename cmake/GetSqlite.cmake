# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "get sqlite ...")

# sqlite version: https://www.sqlite.org/chronology.html
set(sqlite_DOWNLOAD_URL
    "https://www.sqlite.org/2023/sqlite-amalgamation-3420000.zip"
    CACHE STRING "")

if(sqlite_LOCAL_SOURCE)
  FetchContent_Declare(
    sqlite
    SOURCE_DIR ${sqlite_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    sqlite
    URL ${sqlite_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

# Wrap it in a function to restrict the scope of the variables
function(get_sqlite)
  FetchContent_GetProperties(sqlite)
  if(NOT sqlite_POPULATED)
    FetchContent_Populate(sqlite)

    # sqlite lib
    add_library(libsqlite)
    add_library(sqlite::libsqlite ALIAS libsqlite)

    file(GLOB head_files ${sqlite_SOURCE_DIR}/*.h)

    target_sources(libsqlite PRIVATE ${sqlite_SOURCE_DIR}/sqlite3.c)
    target_include_directories(libsqlite PUBLIC $<BUILD_INTERFACE:${sqlite_SOURCE_DIR}>)
    target_sources(libsqlite INTERFACE FILE_SET HEADERS BASE_DIRS ${sqlite_SOURCE_DIR} FILES ${head_files})

    if(UNIX)
      target_link_libraries(libsqlite PUBLIC pthread dl)
    endif()
  endif()
endfunction()

get_sqlite()

# import targets:
# sqlite::libsqlite
