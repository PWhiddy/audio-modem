#ifndef AUDIO_MODEM_TUNNEL_H
#define AUDIO_MODEM_TUNNEL_H

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

typedef struct tunnel tunnel_t;

int tunnel_open(tunnel_t **out, int gateway, int configure,
                char *err, size_t err_size);
int tunnel_fd(const tunnel_t *tunnel);
const char *tunnel_name(const tunnel_t *tunnel);
ssize_t tunnel_read(tunnel_t *tunnel, uint8_t *packet, size_t capacity);
ssize_t tunnel_write(tunnel_t *tunnel, const uint8_t *packet, size_t length);
void tunnel_close(tunnel_t *tunnel);

#endif
