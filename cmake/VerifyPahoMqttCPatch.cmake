if(NOT DEFINED PAHO_SOURCE_DIR)
  message(FATAL_ERROR "PAHO_SOURCE_DIR is required")
endif()

set(socket_file "${PAHO_SOURCE_DIR}/src/Socket.c")
if(NOT EXISTS "${socket_file}")
  message(FATAL_ERROR "Paho Socket.c was not found: ${socket_file}")
endif()

file(READ "${socket_file}" socket_source)
string(FIND "${socket_source}" "int poll_timeout_ms = (mod_s.write_pending->count > 0) ? 0 : timeout_ms;" patched_at)
if(patched_at EQUAL -1)
  message(FATAL_ERROR "Paho pending-write polling patch is missing")
endif()

message(STATUS "Paho pending-write polling patch is present")
