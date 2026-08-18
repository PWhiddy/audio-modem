#ifndef AUDIO_MODEM_PACKET_H
#define AUDIO_MODEM_PACKET_H

#include <stddef.h>
#include <stdint.h>

#include "modem.h"

#define PACKET_MAX 1500u
#define PACKET_QUEUE_CAP 32u

typedef struct {
    size_t len;
    uint8_t data[PACKET_MAX];
} packet_t;

typedef struct {
    packet_t items[PACKET_QUEUE_CAP];
    unsigned head;
    unsigned count;
} packet_queue_t;

typedef struct {
    packet_t packet;
    size_t offset;
    size_t staged;
    uint16_t id;
    int active;
} packet_sender_t;

typedef struct {
    packet_t packet;
    size_t received;
    uint16_t id;
    uint16_t last_complete;
    int active;
} packet_receiver_t;

void packet_queue_init(packet_queue_t *queue);
int packet_queue_push(packet_queue_t *queue, const uint8_t *data, size_t len);
int packet_queue_pop(packet_queue_t *queue, packet_t *packet);

void packet_sender_init(packet_sender_t *sender);
/* Stages (but does not commit) the next fragment. */
size_t packet_sender_fragment(packet_sender_t *sender, packet_queue_t *queue,
                              uint8_t out[MODEM_PAYLOAD_MAX],
                              size_t payload_limit);
void packet_sender_commit(packet_sender_t *sender);

void packet_receiver_init(packet_receiver_t *receiver);
/* Returns 1 when packet is complete, 0 for an accepted partial, -1 if invalid. */
int packet_receiver_add(packet_receiver_t *receiver, const uint8_t *fragment,
                        size_t len, packet_t *packet);

int packet_self_test(void);

#endif
