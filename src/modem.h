#ifndef AUDIO_MODEM_MODEM_H
#define AUDIO_MODEM_MODEM_H

#include <stddef.h>
#include <stdint.h>

#define MODEM_MIN_HZ 2000u
#define MODEM_MAX_HZ 12000u
#define MODEM_BOOTSTRAP_MAX_HZ 6000u
#define MODEM_DEFAULT_MAX_HZ 8000u
#define MODEM_PAYLOAD_MAX 384u

enum modem_frame_type {
    MODEM_HELLO = 1,
    MODEM_OFFER = 2,
    MODEM_CONFIRM = 3,
    MODEM_READY = 4,
    MODEM_REQUEST = 5,
    MODEM_RESPONSE = 6
};

typedef struct {
    uint8_t type;
    uint8_t flags;
    uint32_t session;
    uint16_t seq;
    uint16_t ack;
    uint16_t band_low;
    uint16_t band_high;
    uint16_t payload_len;
    uint8_t payload[MODEM_PAYLOAD_MAX];
} modem_frame_t;

typedef struct modem_decoder modem_decoder_t;

typedef struct {
    double peak_sync;
    unsigned candidates;
    unsigned rejected;
} modem_activity_t;

modem_decoder_t *modem_decoder_create(unsigned low_hz, unsigned high_hz);
void modem_decoder_destroy(modem_decoder_t *decoder);
int modem_decoder_set_band(modem_decoder_t *decoder, unsigned low_hz,
                           unsigned high_hz);
/* Returns 1 with a frame, 0 when more samples are needed, and -1 on overflow. */
int modem_decoder_feed(modem_decoder_t *decoder, const float *samples,
                       size_t count, modem_frame_t *frame);
/* Returns and clears receive activity accumulated since the previous call. */
void modem_decoder_take_activity(modem_decoder_t *decoder,
                                 modem_activity_t *activity);

int modem_encode(const modem_frame_t *frame, unsigned low_hz, unsigned high_hz,
                 float **samples, size_t *count);
size_t modem_burst_samples(unsigned low_hz, unsigned high_hz);
void modem_free_samples(float *samples);

/* Deterministic DSP/codec tests, with diagnostic output on failure. */
int modem_self_test(void);

#endif
