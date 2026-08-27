if(NOT DEFINED PAHO_SOURCE_DIR)
  message(FATAL_ERROR "PAHO_SOURCE_DIR is required")
endif()

set(socket_file "${PAHO_SOURCE_DIR}/src/Socket.c")
if(NOT EXISTS "${socket_file}")
  message(FATAL_ERROR "Paho Socket.c was not found: ${socket_file}")
endif()

file(READ "${socket_file}" socket_source)

set(already_patched "\t\tint poll_timeout_ms = (mod_s.write_pending->count > 0) ? 0 : timeout_ms;")
string(FIND "${socket_source}" "${already_patched}" already_patched_at)
if(NOT already_patched_at EQUAL -1)
  message(STATUS "Paho pending-write polling patch already applied")
  return()
endif()

set(old_block
    "\t\t/* Prevent performance issue by unlocking the socket_mutex while waiting for a ready socket. */\n\t\tPaho_thread_unlock_mutex(mutex);\n\t\t*rc = poll(mod_s.saved.fds_read, mod_s.saved.nfds, timeout_ms);"
)
set(new_block
    "\t\tint poll_timeout_ms = (mod_s.write_pending->count > 0) ? 0 : timeout_ms;\n\n\t\t/* Prevent performance issue by unlocking the socket_mutex while waiting for a ready socket. */\n\t\tPaho_thread_unlock_mutex(mutex);\n\t\t/* A partial write must be retried without waiting for the read timeout. */\n\t\t*rc = poll(mod_s.saved.fds_read, mod_s.saved.nfds, poll_timeout_ms);"
)
string(FIND "${socket_source}" "${old_block}" old_block_at)
if(old_block_at EQUAL -1)
  message(FATAL_ERROR "Unsupported Paho Socket.c: expected poll timeout block was not found")
endif()

string(REPLACE "${old_block}" "${new_block}" socket_source "${socket_source}")
file(WRITE "${socket_file}" "${socket_source}")
message(STATUS "Applied Paho pending-write polling patch")
