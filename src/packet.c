#include "packet.h"

#include <stdio.h>
#include <string.h>

#define FRAGMENT_HEADER 8u
#define FRAGMENT_DATA_MAX (MODEM_PAYLOAD_MAX - FRAGMENT_HEADER)

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static void put_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)value;
}

void packet_queue_init(packet_queue_t *queue)
{
    memset(queue, 0, sizeof(*queue));
}

int packet_queue_push(packet_queue_t *queue, const uint8_t *data, size_t len)
{
    unsigned slot;

    if (len == 0 || len > PACKET_MAX || queue->count == PACKET_QUEUE_CAP)
        return -1;
    slot = (queue->head + queue->count) % PACKET_QUEUE_CAP;
    memcpy(queue->items[slot].data, data, len);
    queue->items[slot].len = len;
    ++queue->count;
    return 0;
}

int packet_queue_pop(packet_queue_t *queue, packet_t *packet)
{
    if (queue->count == 0)
        return 0;
    *packet = queue->items[queue->head];
    queue->head = (queue->head + 1) % PACKET_QUEUE_CAP;
    --queue->count;
    return 1;
}

void packet_sender_init(packet_sender_t *sender)
{
    memset(sender, 0, sizeof(*sender));
    sender->id = 1;
}

size_t packet_sender_fragment(packet_sender_t *sender, packet_queue_t *queue,
                              uint8_t out[MODEM_PAYLOAD_MAX],
                              size_t payload_limit)
{
    size_t chunk;

    if (payload_limit < FRAGMENT_HEADER + 1u ||
        payload_limit > MODEM_PAYLOAD_MAX)
        return 0;

    if (!sender->active) {
        if (!packet_queue_pop(queue, &sender->packet))
            return 0;
        sender->offset = 0;
        sender->active = 1;
    }
    chunk = sender->packet.len - sender->offset;
    if (chunk > payload_limit - FRAGMENT_HEADER)
        chunk = payload_limit - FRAGMENT_HEADER;
    sender->staged = chunk;
    put_u16(out, sender->id);
    put_u16(out + 2, (uint16_t)sender->offset);
    put_u16(out + 4, (uint16_t)sender->packet.len);
    put_u16(out + 6, (uint16_t)chunk);
    memcpy(out + FRAGMENT_HEADER, sender->packet.data + sender->offset, chunk);
    return chunk + FRAGMENT_HEADER;
}

void packet_sender_commit(packet_sender_t *sender)
{
    size_t chunk;

    if (!sender->active)
        return;
    chunk = sender->staged;
    if (chunk == 0 || chunk > sender->packet.len - sender->offset)
        return;
    sender->offset += chunk;
    sender->staged = 0;
    if (sender->offset == sender->packet.len) {
        sender->active = 0;
        sender->offset = 0;
        if (++sender->id == 0)
            sender->id = 1;
    }
}

void packet_receiver_init(packet_receiver_t *receiver)
{
    memset(receiver, 0, sizeof(*receiver));
}

int packet_receiver_add(packet_receiver_t *receiver, const uint8_t *fragment,
                        size_t len, packet_t *packet)
{
    uint16_t id, offset, total, chunk;

    if (len < FRAGMENT_HEADER)
        return -1;
    id = get_u16(fragment);
    offset = get_u16(fragment + 2);
    total = get_u16(fragment + 4);
    chunk = get_u16(fragment + 6);
    if (id == 0 || total == 0 || total > PACKET_MAX ||
        chunk > FRAGMENT_DATA_MAX || len != (size_t)chunk + FRAGMENT_HEADER ||
        (size_t)offset + chunk > total)
        return -1;
    if (!receiver->active && id == receiver->last_complete)
        return 0;

    if (!receiver->active || receiver->id != id) {
        if (offset != 0)
            return -1;
        receiver->active = 1;
        receiver->id = id;
        receiver->received = 0;
        receiver->packet.len = total;
    }
    /* Duplicates are harmless; gaps indicate a broken link transaction. */
    if (offset < receiver->received)
        return 0;
    if (offset != receiver->received || total != receiver->packet.len) {
        receiver->active = 0;
        return -1;
    }
    memcpy(receiver->packet.data + offset, fragment + FRAGMENT_HEADER, chunk);
    receiver->received += chunk;
    if (receiver->received == receiver->packet.len) {
        *packet = receiver->packet;
        receiver->active = 0;
        receiver->last_complete = id;
        return 1;
    }
    return 0;
}

int packet_self_test(void)
{
    packet_queue_t queue;
    packet_sender_t sender;
    packet_receiver_t receiver;
    packet_t result;
    uint8_t source[PACKET_MAX];
    uint8_t fragment[MODEM_PAYLOAD_MAX];
    static const size_t limits[] = {16u, 32u, 64u, 96u};
    size_t len, i;
    unsigned fragment_index = 0;
    int status = 0;
    int complete = 0;

    for (i = 0; i < sizeof(source); ++i)
        source[i] = (uint8_t)(i * 37u + 11u);
    packet_queue_init(&queue);
    packet_sender_init(&sender);
    packet_receiver_init(&receiver);
    if (packet_queue_push(&queue, source, sizeof(source)) != 0)
        status = -1;
    while (!status && (sender.active || queue.count)) {
        size_t limit = limits[fragment_index %
                              (sizeof(limits) / sizeof(limits[0]))];
        len = packet_sender_fragment(&sender, &queue, fragment, limit);
        ++fragment_index;
        if (len == 0)
            status = -1;
        else {
            int added = packet_receiver_add(&receiver, fragment, len, &result);
            int duplicate = packet_receiver_add(&receiver, fragment, len,
                                                &result);
            packet_sender_commit(&sender);
            if (duplicate != 0)
                status = -1;
            if (added == 1) {
                if (result.len != sizeof(source) ||
                    memcmp(result.data, source, sizeof(source)) != 0)
                    status = -1;
                else
                    complete = 1;
            } else if (added < 0) {
                status = -1;
            }
        }
    }
    if (status != 0 || receiver.active || !complete) {
        fprintf(stderr, "packet fragmentation self-test failed\n");
        return -1;
    }
    return 0;
}
