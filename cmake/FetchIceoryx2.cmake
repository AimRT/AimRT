# FetchIceoryx2.cmake
# Automatically fetches and builds iceoryx2 C++ bindings
#
# Usage: include(FetchIceoryx2) in your CMakeLists.txt
# 
# This module will:
# 1. Check for Rust/Cargo installation
# 2. Fetch iceoryx2 source from GitHub
# 3. Build the C++ bindings using cargo
# 4. Export include and library paths

include(FetchContent)
include(ExternalProject)

# ============================================================================
# Check for Rust toolchain
# ============================================================================

find_program(CARGO_EXECUTABLE cargo)
find_program(RUSTC_EXECUTABLE rustc)

if(NOT CARGO_EXECUTABLE OR NOT RUSTC_EXECUTABLE)
  message(FATAL_ERROR "
================================================================================
Rust/Cargo not found! Iceoryx2 plugin requires Rust toolchain.

Install Rust:
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

After installation, add to PATH and use nightly:
  source ~/.cargo/env
  rustup default nightly

Then re-run cmake.
================================================================================
")
endif()

# Check Rust version
execute_process(
  COMMAND ${RUSTC_EXECUTABLE} --version
  OUTPUT_VARIABLE RUSTC_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
message(STATUS "Found Rust: ${RUSTC_VERSION}")

# ============================================================================
# Fetch iceoryx2 source
# ============================================================================

set(ICEORYX2_VERSION "v0.8.0" CACHE STRING "Iceoryx2 version to fetch")

FetchContent_Declare(
  iceoryx2_src
  GIT_REPOSITORY https://github.com/eclipse-iceoryx/iceoryx2.git
  GIT_TAG        ${ICEORYX2_VERSION}
  GIT_SHALLOW    TRUE
  GIT_PROGRESS   TRUE
)

message(STATUS "Fetching iceoryx2 ${ICEORYX2_VERSION}...")
FetchContent_MakeAvailable(iceoryx2_src)

# ============================================================================
# Build iceoryx2 C++ bindings
# ============================================================================

set(ICEORYX2_BUILD_DIR ${iceoryx2_src_SOURCE_DIR}/target/release)
set(ICEORYX2_CXX_DIR ${iceoryx2_src_SOURCE_DIR}/iceoryx2-cxx)

# Determine library extension based on platform
if(WIN32)
  set(ICEORYX2_LIB_NAME "iceoryx2_ffi.lib")
elseif(APPLE)
  set(ICEORYX2_LIB_NAME "libiceoryx2_ffi.a")
else()
  set(ICEORYX2_LIB_NAME "libiceoryx2_ffi.a")
endif()

# Use ExternalProject to build with cargo
ExternalProject_Add(iceoryx2_cargo_build
  SOURCE_DIR        ${iceoryx2_src_SOURCE_DIR}
  CONFIGURE_COMMAND ""
  BUILD_COMMAND     ${CARGO_EXECUTABLE} build --release -p iceoryx2-ffi-c -p iceoryx2-cxx
  BUILD_IN_SOURCE   TRUE
  BUILD_BYPRODUCTS  ${ICEORYX2_BUILD_DIR}/${ICEORYX2_LIB_NAME}
  INSTALL_COMMAND   ""
  LOG_BUILD         TRUE
)

# ============================================================================
# Create imported library target
# ============================================================================

add_library(iceoryx2::cxx STATIC IMPORTED GLOBAL)
set_target_properties(iceoryx2::cxx PROPERTIES
  IMPORTED_LOCATION ${ICEORYX2_BUILD_DIR}/${ICEORYX2_LIB_NAME}
  INTERFACE_INCLUDE_DIRECTORIES ${ICEORYX2_CXX_DIR}/include
)
add_dependencies(iceoryx2::cxx iceoryx2_cargo_build)

# Export variables for use in other CMakeLists
set(ICEORYX2_INCLUDE_DIRS ${ICEORYX2_CXX_DIR}/include CACHE PATH "Iceoryx2 include directories")
set(ICEORYX2_LIBRARIES iceoryx2::cxx CACHE STRING "Iceoryx2 library target")

message(STATUS "Iceoryx2 include: ${ICEORYX2_INCLUDE_DIRS}")
message(STATUS "Iceoryx2 library: ${ICEORYX2_BUILD_DIR}/${ICEORYX2_LIB_NAME}")
