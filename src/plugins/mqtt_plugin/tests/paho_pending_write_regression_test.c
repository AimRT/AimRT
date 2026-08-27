#include "MQTTAsync.h"
#include "Socket.h"
#include "mutex_type.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

extern mutex_type socket_mutex;
extern int Socket_addSocket(SOCKET socket);

static double MonotonicSeconds(void) {
  struct timespec ts;
  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
    perror("clock_gettime");
    exit(EXIT_FAILURE);
  }
  return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

int main(void) {
  int sockets[2] = {-1, -1};
  const size_t payload_size = 4 * 1024 * 1024;
  char* payload = NULL;
  PacketBuffers empty_buffers = {0};
  int rc = 0;
  int ready_rc = 0;
  double elapsed;

  MQTTAsync_init_options init_options = MQTTAsync_init_options_initializer;
  MQTTAsync_global_init(&init_options);
  Socket_outInitialize();

  if (socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) != 0) {
    perror("socketpair");
    return EXIT_FAILURE;
  }

  if (fcntl(sockets[0], F_SETFL, O_NONBLOCK) != 0) {
    perror("fcntl");
    return EXIT_FAILURE;
  }

  char fill[64 * 1024];
  memset(fill, 'f', sizeof(fill));
  while (write(sockets[0], fill, sizeof(fill)) > 0) {
  }
  if (errno != EAGAIN && errno != EWOULDBLOCK) {
    perror("write");
    return EXIT_FAILURE;
  }

  if (Socket_addSocket(sockets[0]) != 0) {
    fprintf(stderr, "Socket_addSocket failed\n");
    return EXIT_FAILURE;
  }

  payload = malloc(payload_size);
  if (payload == NULL) {
    perror("malloc");
    return EXIT_FAILURE;
  }
  memset(payload, 'x', payload_size);

  rc = Socket_putdatas(sockets[0], payload, payload_size, empty_buffers);
  if (rc != TCPSOCKET_INTERRUPTED || Socket_noPendingWrites(sockets[0])) {
    fprintf(stderr, "expected a pending short write, rc=%d pending=%d errno=%d\n", rc,
            !Socket_noPendingWrites(sockets[0]), errno);
    return EXIT_FAILURE;
  }

  elapsed = MonotonicSeconds();
  (void)Socket_getReadySocket(0, 1000, socket_mutex, &ready_rc);
  elapsed = MonotonicSeconds() - elapsed;

  Socket_close(sockets[0]);
  close(sockets[1]);
  Socket_outTerminate();

  if (ready_rc != 0) {
    fprintf(stderr, "Socket_getReadySocket failed: rc=%d\n", ready_rc);
    return EXIT_FAILURE;
  }
  if (elapsed >= 0.5) {
    fprintf(stderr, "pending write polling took %.3f seconds\n", elapsed);
    return EXIT_FAILURE;
  }

  printf("pending write polling returned in %.3f seconds\n", elapsed);
  return EXIT_SUCCESS;
}
