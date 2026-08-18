#include "modem.h"

#include "audio.h"

#include <complex.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FFT_N 256u
#define CP_N 64u
#define SYMBOL_N (FFT_N + CP_N)
#define LEAD_N 480u
#define TAIL_N 240u
#define DETECT_LOOKAHEAD 32u
#define HEADER_N 20u
#define RAW_N (HEADER_N + MODEM_PAYLOAD_MAX + 4u)
#define RAW_BITS (RAW_N * 8u)
#define CODE_STEPS (RAW_BITS + 6u)
#define CODE_BITS (CODE_STEPS * 2u)
#define RX_CAPACITY (AUDIO_SAMPLE_RATE * 6u)
#define PI 3.14159265358979323846

typedef struct {
    unsigned low_bin;
    unsigned high_bin;
    unsigned data_carriers;
    unsigned symbols;
} band_t;

struct modem_decoder {
    band_t band;
    float *rx;
    size_t length;
    size_t search_position;
    double peak_sync;
    unsigned candidates;
    unsigned rejected;
};

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint32_t get_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}

static void put_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)value;
}

static void put_u32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value >> 24);
    p[1] = (uint8_t)(value >> 16);
    p[2] = (uint8_t)(value >> 8);
    p[3] = (uint8_t)value;
}

static uint32_t crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xffffffffu;
    size_t i;
    unsigned bit;

    for (i = 0; i < length; ++i) {
        crc ^= data[i];
        for (bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

static unsigned parity(unsigned value)
{
    value ^= value >> 4;
    value &= 15u;
    return (0x6996u >> value) & 1u;
}

static int make_band(unsigned low_hz, unsigned high_hz, band_t *band)
{
    unsigned k, pilots = 0;

    if (low_hz < MODEM_MIN_HZ || high_hz > MODEM_MAX_HZ ||
        low_hz + 1500u > high_hz)
        return -1;
    band->low_bin = (low_hz * FFT_N + AUDIO_SAMPLE_RATE - 1u) /
                    AUDIO_SAMPLE_RATE;
    band->high_bin = (high_hz * FFT_N) / AUDIO_SAMPLE_RATE;
    if (band->low_bin < 1u)
        band->low_bin = 1u;
    if (band->high_bin >= FFT_N / 2u)
        band->high_bin = FFT_N / 2u - 1u;
    if (band->high_bin <= band->low_bin)
        return -1;
    for (k = band->low_bin; k <= band->high_bin; ++k)
        if ((k - band->low_bin) % 8u == 0u)
            ++pilots;
    band->data_carriers = band->high_bin - band->low_bin + 1u - pilots;
    if (band->data_carriers < 6u)
        return -1;
    band->symbols = (CODE_BITS + band->data_carriers * 4u - 1u) /
                    (band->data_carriers * 4u);
    return 0;
}

static int is_pilot(const band_t *band, unsigned bin)
{
    return (bin - band->low_bin) % 8u == 0u;
}

static double pn_value(unsigned bin, unsigned symbol)
{
    uint32_t x = 0x9e3779b9u ^ (bin * 0x85ebca6bu) ^
                 (symbol * 0xc2b2ae35u);
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    return (x & 1u) ? 1.0 : -1.0;
}

static void fft(double complex *x, int inverse)
{
    unsigned i, j, len;

    for (i = 1, j = 0; i < FFT_N; ++i) {
        unsigned bit = FFT_N >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j) {
            double complex tmp = x[i];
            x[i] = x[j];
            x[j] = tmp;
        }
    }
    for (len = 2; len <= FFT_N; len <<= 1) {
        double angle = (inverse ? 2.0 : -2.0) * PI / len;
        double complex wlen = cos(angle) + I * sin(angle);
        unsigned base;
        for (base = 0; base < FFT_N; base += len) {
            double complex w = 1.0;
            for (j = 0; j < len / 2; ++j) {
                double complex u = x[base + j];
                double complex v = x[base + j + len / 2] * w;
                x[base + j] = u + v;
                x[base + j + len / 2] = u - v;
                w *= wlen;
            }
        }
    }
    if (inverse)
        for (i = 0; i < FFT_N; ++i)
            x[i] /= FFT_N;
}

static void emit_symbol(const double complex *frequency, float *output)
{
    double complex time[FFT_N];
    unsigned i;

    memcpy(time, frequency, sizeof(time));
    fft(time, 1);
    for (i = 0; i < CP_N; ++i)
        output[i] = (float)creal(time[FFT_N - CP_N + i]);
    for (i = 0; i < FFT_N; ++i)
        output[CP_N + i] = (float)creal(time[i]);
}

static void training_symbol(const band_t *band, float output[SYMBOL_N])
{
    double complex frequency[FFT_N];
    unsigned k;

    memset(frequency, 0, sizeof(frequency));
    for (k = band->low_bin; k <= band->high_bin; ++k) {
        frequency[k] = pn_value(k, 0);
        frequency[FFT_N - k] = conj(frequency[k]);
    }
    emit_symbol(frequency, output);
}

static void serialize_frame(const modem_frame_t *frame, uint8_t raw[RAW_N])
{
    size_t content_length;
    uint32_t crc;

    memset(raw, 0, RAW_N);
    raw[0] = 0xa7;
    raw[1] = 0x3c;
    raw[2] = 1;
    raw[3] = frame->type;
    raw[4] = frame->flags;
    put_u32(raw + 6, frame->session);
    put_u16(raw + 10, frame->seq);
    put_u16(raw + 12, frame->ack);
    put_u16(raw + 14, frame->band_low);
    put_u16(raw + 16, frame->band_high);
    put_u16(raw + 18, frame->payload_len);
    if (frame->payload_len <= MODEM_PAYLOAD_MAX)
        memcpy(raw + HEADER_N, frame->payload, frame->payload_len);
    content_length = HEADER_N + frame->payload_len;
    crc = crc32(raw, content_length);
    put_u32(raw + content_length, crc);
}

static int parse_frame(const uint8_t raw[RAW_N], modem_frame_t *frame)
{
    uint16_t payload_len;

    if (raw[0] != 0xa7 || raw[1] != 0x3c || raw[2] != 1 ||
        raw[3] < MODEM_HELLO || raw[3] > MODEM_RESPONSE)
        return -1;
    payload_len = get_u16(raw + 18);
    if (payload_len > MODEM_PAYLOAD_MAX ||
        get_u32(raw + HEADER_N + payload_len) !=
            crc32(raw, HEADER_N + payload_len))
        return -1;
    memset(frame, 0, sizeof(*frame));
    frame->type = raw[3];
    frame->flags = raw[4];
    frame->session = get_u32(raw + 6);
    frame->seq = get_u16(raw + 10);
    frame->ack = get_u16(raw + 12);
    frame->band_low = get_u16(raw + 14);
    frame->band_high = get_u16(raw + 16);
    frame->payload_len = payload_len;
    memcpy(frame->payload, raw + HEADER_N, payload_len);
    return 0;
}

static void convolutional_encode(const uint8_t raw[RAW_N], uint8_t code[CODE_BITS])
{
    unsigned state = 0, step;

    for (step = 0; step < CODE_STEPS; ++step) {
        unsigned input = step < RAW_BITS
                             ? (raw[step / 8u] >> (7u - step % 8u)) & 1u
                             : 0u;
        unsigned reg = ((state << 1) | input) & 0x7fu;
        code[step * 2u] = (uint8_t)parity(reg & 0x79u); /* K=7, 171 octal */
        code[step * 2u + 1u] = (uint8_t)parity(reg & 0x5bu); /* 133 octal */
        state = reg & 0x3fu;
    }
}

static int convolutional_decode(const float llr[CODE_BITS], uint8_t raw[RAW_N])
{
    float metric[64], next_metric[64];
    uint8_t *trace;
    unsigned step, state, input;

    trace = malloc(CODE_STEPS * 64u);
    if (!trace)
        return -1;
    for (state = 0; state < 64u; ++state)
        metric[state] = state == 0 ? 0.0f : 1.0e30f;
    for (step = 0; step < CODE_STEPS; ++step) {
        for (state = 0; state < 64u; ++state)
            next_metric[state] = 1.0e30f;
        for (state = 0; state < 64u; ++state) {
            if (metric[state] > 1.0e29f)
                continue;
            for (input = 0; input < 2u; ++input) {
                unsigned reg = ((state << 1) | input) & 0x7fu;
                unsigned next = reg & 0x3fu;
                unsigned a = parity(reg & 0x79u);
                unsigned b = parity(reg & 0x5bu);
                float branch = (a ? -llr[step * 2u] : llr[step * 2u]) +
                               (b ? -llr[step * 2u + 1u]
                                  : llr[step * 2u + 1u]);
                float candidate = metric[state] + branch;
                if (candidate < next_metric[next]) {
                    next_metric[next] = candidate;
                    trace[step * 64u + next] =
                        (uint8_t)(state | (input << 6));
                }
            }
        }
        memcpy(metric, next_metric, sizeof(metric));
    }

    memset(raw, 0, RAW_N);
    state = 0;
    for (step = CODE_STEPS; step-- > 0;) {
        uint8_t choice = trace[step * 64u + state];
        input = choice >> 6;
        if (step < RAW_BITS)
            raw[step / 8u] |= (uint8_t)(input << (7u - step % 8u));
        state = choice & 63u;
    }
    free(trace);
    return 0;
}

static double qam_level(unsigned sign, unsigned inner)
{
    if (!sign)
        return inner ? -1.0 : -3.0;
    return inner ? 1.0 : 3.0;
}

int modem_encode(const modem_frame_t *frame, unsigned low_hz, unsigned high_hz,
                 float **samples, size_t *count)
{
    band_t band;
    uint8_t raw[RAW_N], code[CODE_BITS];
    float training[SYMBOL_N];
    float *output;
    size_t total, position, code_pos = 0;
    unsigned symbol, k;
    double peak = 0.0;

    if (!frame || !samples || !count ||
        frame->payload_len > MODEM_PAYLOAD_MAX ||
        make_band(low_hz, high_hz, &band) != 0)
        return -1;
    total = LEAD_N + 2u * SYMBOL_N + (size_t)band.symbols * SYMBOL_N + TAIL_N;
    output = calloc(total, sizeof(*output));
    if (!output)
        return -1;

    serialize_frame(frame, raw);
    convolutional_encode(raw, code);
    training_symbol(&band, training);
    position = LEAD_N;
    memcpy(output + position, training, sizeof(training));
    position += SYMBOL_N;
    memcpy(output + position, training, sizeof(training));
    position += SYMBOL_N;

    for (symbol = 0; symbol < band.symbols; ++symbol) {
        double complex frequency[FFT_N];
        memset(frequency, 0, sizeof(frequency));
        for (k = band.low_bin; k <= band.high_bin; ++k) {
            double complex value;
            if (is_pilot(&band, k)) {
                value = pn_value(k, symbol + 1u);
            } else {
                unsigned bits[4] = {0, 0, 0, 0};
                unsigned b;
                for (b = 0; b < 4u && code_pos < CODE_BITS; ++b)
                    bits[b] = code[code_pos++];
                value = (qam_level(bits[0], bits[1]) +
                         I * qam_level(bits[2], bits[3])) /
                        sqrt(10.0);
            }
            frequency[k] = value;
            frequency[FFT_N - k] = conj(value);
        }
        emit_symbol(frequency, output + position);
        position += SYMBOL_N;
    }

    for (position = LEAD_N; position < total - TAIL_N; ++position) {
        double absolute = fabs(output[position]);
        if (absolute > peak)
            peak = absolute;
    }
    if (peak > 0.0) {
        float scale = (float)(0.72 / peak);
        for (position = LEAD_N; position < total - TAIL_N; ++position)
            output[position] *= scale;
    }
    *samples = output;
    *count = total;
    return 0;
}

size_t modem_burst_samples(unsigned low_hz, unsigned high_hz)
{
    band_t band;
    if (make_band(low_hz, high_hz, &band) != 0)
        return 0;
    return LEAD_N + 2u * SYMBOL_N + (size_t)band.symbols * SYMBOL_N + TAIL_N;
}

void modem_free_samples(float *samples)
{
    free(samples);
}

static double correlation(const float *a, const float *b, size_t length)
{
    double dot = 0.0, ea = 0.0, eb = 0.0;
    size_t i;

    for (i = 0; i < length; ++i) {
        dot += (double)a[i] * b[i];
        ea += (double)a[i] * a[i];
        eb += (double)b[i] * b[i];
    }
    if (ea < 1.0e-9 || eb < 1.0e-9)
        return 0.0;
    return fabs(dot) / sqrt(ea * eb);
}

static int decode_at(const modem_decoder_t *decoder, size_t start,
                     modem_frame_t *frame)
{
    const band_t *band = &decoder->band;
    double complex channel[FFT_N];
    float llr[CODE_BITS];
    uint8_t raw[RAW_N];
    size_t llr_pos = 0;
    long sample_adjustment = 0;
    unsigned training_index, symbol, k;

    memset(channel, 0, sizeof(channel));
    for (training_index = 0; training_index < 2u; ++training_index) {
        double complex time[FFT_N];
        size_t base = start + training_index * SYMBOL_N + CP_N;
        unsigned i;
        for (i = 0; i < FFT_N; ++i)
            time[i] = decoder->rx[base + i];
        fft(time, 0);
        for (k = band->low_bin; k <= band->high_bin; ++k)
            channel[k] += time[k] / pn_value(k, 0) / 2.0;
    }

    for (symbol = 0; symbol < band->symbols; ++symbol) {
        double complex time[FFT_N];
        double complex slope_sum = 0.0;
        double complex phase_sum = 0.0;
        double complex rotation;
        double timing = 0.0;
        double complex previous_pilot = 0.0;
        int have_previous_pilot = 0;
        size_t base = start + 2u * SYMBOL_N + symbol * SYMBOL_N + CP_N +
                      sample_adjustment;
        unsigned i;

        for (i = 0; i < FFT_N; ++i)
            time[i] = decoder->rx[base + i];
        fft(time, 0);
        for (k = band->low_bin; k <= band->high_bin; ++k) {
            double complex pilot;
            double magnitude;
            if (!is_pilot(band, k) || cabs(channel[k]) <= 1.0e-8)
                continue;
            pilot = time[k] / channel[k] * pn_value(k, symbol + 1u);
            if (have_previous_pilot) {
                double complex difference = pilot * conj(previous_pilot);
                magnitude = cabs(difference);
                if (magnitude > 1.0e-8)
                    slope_sum += difference / magnitude;
            }
            previous_pilot = pilot;
            have_previous_pilot = 1;
        }
        if (cabs(slope_sum) > 1.0e-8)
            timing = -carg(slope_sum) * FFT_N / (2.0 * PI * 8.0);
        for (k = band->low_bin; k <= band->high_bin; ++k) {
            if (is_pilot(band, k) && cabs(channel[k]) > 1.0e-8) {
                double angle = 2.0 * PI * k * timing / FFT_N;
                phase_sum += time[k] / channel[k] *
                             (cos(angle) + I * sin(angle)) *
                             pn_value(k, symbol + 1u);
            }
        }
        rotation = cabs(phase_sum) > 1.0e-8
                       ? conj(phase_sum) / cabs(phase_sum)
                       : 1.0;
        for (k = band->low_bin; k <= band->high_bin; ++k) {
            double complex value;
            double x, y;
            if (is_pilot(band, k) || llr_pos >= CODE_BITS)
                continue;
            if (cabs(channel[k]) < 1.0e-8)
                value = 0.0;
            else {
                double angle = 2.0 * PI * k * timing / FFT_N;
                value = time[k] / channel[k] *
                        (cos(angle) + I * sin(angle)) * rotation;
            }
            x = creal(value) * sqrt(10.0);
            y = cimag(value) * sqrt(10.0);
            llr[llr_pos++] = (float)x;
            llr[llr_pos++] = (float)(2.0 - fabs(x));
            llr[llr_pos++] = (float)y;
            llr[llr_pos++] = (float)(2.0 - fabs(y));
        }
        if (timing <= -0.5 || timing >= 0.5) {
            sample_adjustment += lround(timing);
            if (sample_adjustment < -(long)DETECT_LOOKAHEAD)
                sample_adjustment = -(long)DETECT_LOOKAHEAD;
            if (sample_adjustment > (long)DETECT_LOOKAHEAD)
                sample_adjustment = (long)DETECT_LOOKAHEAD;
        }
    }
    if (llr_pos != CODE_BITS || convolutional_decode(llr, raw) != 0)
        return -1;
    return parse_frame(raw, frame);
}

modem_decoder_t *modem_decoder_create(unsigned low_hz, unsigned high_hz)
{
    modem_decoder_t *decoder = calloc(1, sizeof(*decoder));

    if (!decoder)
        return NULL;
    decoder->rx = malloc(RX_CAPACITY * sizeof(*decoder->rx));
    if (!decoder->rx || make_band(low_hz, high_hz, &decoder->band) != 0) {
        modem_decoder_destroy(decoder);
        return NULL;
    }
    return decoder;
}

void modem_decoder_destroy(modem_decoder_t *decoder)
{
    if (decoder) {
        free(decoder->rx);
        free(decoder);
    }
}

int modem_decoder_set_band(modem_decoder_t *decoder, unsigned low_hz,
                           unsigned high_hz)
{
    band_t band;

    if (!decoder || make_band(low_hz, high_hz, &band) != 0)
        return -1;
    decoder->band = band;
    decoder->length = 0;
    decoder->search_position = 0;
    return 0;
}

void modem_decoder_take_activity(modem_decoder_t *decoder,
                                 modem_activity_t *activity)
{
    if (!activity)
        return;
    memset(activity, 0, sizeof(*activity));
    if (!decoder)
        return;
    activity->peak_sync = decoder->peak_sync;
    activity->candidates = decoder->candidates;
    activity->rejected = decoder->rejected;
    decoder->peak_sync = 0.0;
    decoder->candidates = 0;
    decoder->rejected = 0;
}

static void discard_samples(modem_decoder_t *decoder, size_t count)
{
    if (count >= decoder->length) {
        decoder->length = 0;
        decoder->search_position = 0;
        return;
    }
    memmove(decoder->rx, decoder->rx + count,
            (decoder->length - count) * sizeof(*decoder->rx));
    decoder->length -= count;
    if (decoder->search_position > count)
        decoder->search_position -= count;
    else
        decoder->search_position = 0;
}

int modem_decoder_feed(modem_decoder_t *decoder, const float *samples,
                       size_t count, modem_frame_t *frame)
{
    float training[SYMBOL_N];
    size_t needed, max_start, i, best_start = 0;
    double best = 0.0;

    if (!decoder || !frame || (count && !samples))
        return -1;
    if (count > RX_CAPACITY) {
        samples += count - RX_CAPACITY;
        count = RX_CAPACITY;
        decoder->length = 0;
    } else if (decoder->length + count > RX_CAPACITY) {
        discard_samples(decoder, decoder->length + count - RX_CAPACITY);
    }
    if (count) {
        memcpy(decoder->rx + decoder->length, samples,
               count * sizeof(*samples));
        decoder->length += count;
    }

    needed = 2u * SYMBOL_N + (size_t)decoder->band.symbols * SYMBOL_N;
    if (decoder->length < needed + DETECT_LOOKAHEAD)
        return 0;
    training_symbol(&decoder->band, training);
    max_start = decoder->length - needed - DETECT_LOOKAHEAD;
    for (i = decoder->search_position; i <= max_start; ++i) {
        double first = correlation(training, decoder->rx + i, SYMBOL_N);
        double score;
        if (first > decoder->peak_sync)
            decoder->peak_sync = first;
        if (first < 0.20)
            continue;
        score = (first + correlation(training, decoder->rx + i + SYMBOL_N,
                                     SYMBOL_N)) /
                2.0;
        if (score > decoder->peak_sync)
            decoder->peak_sync = score;
        if (score > best) {
            best = score;
            best_start = i;
        }
    }
    decoder->search_position = max_start + 1u;
    if (best >= 0.55) {
        int result = decode_at(decoder, best_start, frame);
        ++decoder->candidates;
        discard_samples(decoder, best_start + needed);
        if (result == 0)
            return 1;
        ++decoder->rejected;
        return 0;
    }
    if (decoder->length > needed + AUDIO_SAMPLE_RATE)
        discard_samples(decoder, decoder->length - needed);
    return 0;
}

static uint32_t test_random(uint32_t *state)
{
    *state = *state * 1664525u + 1013904223u;
    return *state;
}

static int test_one_band(unsigned low, unsigned high)
{
    modem_frame_t sent, received;
    modem_decoder_t *decoder;
    float *encoded, *channel;
    size_t encoded_count, channel_count, position;
    uint32_t random = 7;
    int got = 0;
    unsigned i;

    memset(&sent, 0, sizeof(sent));
    sent.type = MODEM_REQUEST;
    sent.flags = 1;
    sent.session = 0x1234abcdu;
    sent.seq = 417;
    sent.ack = 416;
    sent.band_low = (uint16_t)low;
    sent.band_high = (uint16_t)high;
    sent.payload_len = MODEM_PAYLOAD_MAX;
    for (i = 0; i < sent.payload_len; ++i)
        sent.payload[i] = (uint8_t)test_random(&random);
    if (modem_encode(&sent, low, high, &encoded, &encoded_count) != 0)
        return -1;
    channel_count = encoded_count + 173u;
    channel = calloc(channel_count, sizeof(*channel));
    decoder = modem_decoder_create(low, high);
    if (!channel || !decoder) {
        free(channel);
        free(encoded);
        modem_decoder_destroy(decoder);
        return -1;
    }
    random = 99;
    for (i = 0; i < channel_count; ++i) {
        double noise = ((int)(test_random(&random) >> 16) - 32768) / 32768.0;
        channel[i] = (float)(noise * 0.006);
    }
    for (i = 0; i < encoded_count; ++i) {
        channel[i + 137u] += encoded[i] * 0.43f;
        if (i >= 7u)
            channel[i + 137u] += encoded[i - 7u] * 0.07f;
    }
    for (position = 0; position < channel_count && !got;) {
        size_t chunk = channel_count - position;
        int result;
        if (chunk > 173u)
            chunk = 173u;
        result = modem_decoder_feed(decoder, channel + position, chunk, &received);
        if (result < 0)
            break;
        got = result;
        position += chunk;
    }
    free(channel);
    free(encoded);
    modem_decoder_destroy(decoder);
    if (!got || received.type != sent.type || received.flags != sent.flags ||
        received.session != sent.session || received.seq != sent.seq ||
        received.ack != sent.ack || received.payload_len != sent.payload_len ||
        memcmp(received.payload, sent.payload, sent.payload_len) != 0) {
        fprintf(stderr, "modem self-test failed for %u-%u Hz\n", low, high);
        return -1;
    }
    return 0;
}

static int test_clock_mismatch(double ratio)
{
    modem_frame_t sent, received;
    modem_decoder_t *decoder = NULL;
    float *encoded = NULL, *channel = NULL;
    size_t encoded_count = 0, resampled_count, channel_count, i, position;
    uint32_t random = 23;
    int got = 0;

    memset(&sent, 0, sizeof(sent));
    sent.type = MODEM_REQUEST;
    sent.session = 0x31415926u;
    sent.seq = 73;
    sent.band_low = MODEM_MIN_HZ;
    sent.band_high = MODEM_MAX_HZ;
    sent.payload_len = MODEM_PAYLOAD_MAX;
    for (i = 0; i < sent.payload_len; ++i)
        sent.payload[i] = (uint8_t)test_random(&random);
    if (modem_encode(&sent, MODEM_MIN_HZ, MODEM_MAX_HZ, &encoded,
                     &encoded_count) != 0)
        goto done;
    resampled_count = (size_t)(encoded_count * ratio);
    channel_count = 137u + resampled_count + 17u + 257u;
    channel = calloc(channel_count, sizeof(*channel));
    decoder = modem_decoder_create(MODEM_MIN_HZ, MODEM_MAX_HZ);
    if (!channel || !decoder)
        goto done;
    for (i = 0; i < channel_count; ++i) {
        double noise = ((int)(test_random(&random) >> 16) - 32768) / 32768.0;
        channel[i] = (float)(noise * 0.004);
    }
    for (i = 0; i < resampled_count; ++i) {
        double source = i / ratio;
        size_t index = (size_t)source;
        double fraction = source - index;
        float sample = index + 1u < encoded_count
                           ? (float)(encoded[index] * (1.0 - fraction) +
                                     encoded[index + 1u] * fraction)
                           : 0.0f;
        channel[137u + i] += sample * 0.43f;
        channel[137u + i + 17u] += sample * 0.09f;
    }
    for (position = 0; position < channel_count && !got;) {
        size_t chunk = channel_count - position;
        int result;
        if (chunk > 173u)
            chunk = 173u;
        result = modem_decoder_feed(decoder, channel + position, chunk,
                                    &received);
        if (result < 0)
            break;
        got = result;
        position += chunk;
    }
    if (got && received.type == sent.type &&
        received.session == sent.session && received.seq == sent.seq &&
        received.payload_len == sent.payload_len &&
        memcmp(received.payload, sent.payload, sent.payload_len) == 0) {
        free(channel);
        modem_free_samples(encoded);
        modem_decoder_destroy(decoder);
        return 0;
    }
done:
    free(channel);
    modem_free_samples(encoded);
    modem_decoder_destroy(decoder);
    fprintf(stderr, "modem clock-mismatch self-test failed at %.0f ppm\n",
            (ratio - 1.0) * 1000000.0);
    return -1;
}

static int test_frame_stream(void)
{
    modem_frame_t sent[2], received;
    modem_decoder_t *decoder;
    float *encoded[2] = {NULL, NULL};
    float *stream = NULL;
    size_t encoded_count[2], stream_count, position = 0;
    unsigned decoded = 0;
    int status = -1;

    memset(sent, 0, sizeof(sent));
    sent[0].type = MODEM_HELLO;
    sent[0].session = 0x10203040u;
    sent[0].seq = 19;
    sent[1].type = MODEM_OFFER;
    sent[1].session = 0x50607080u;
    sent[1].seq = 20;
    if (modem_encode(&sent[0], MODEM_MIN_HZ, MODEM_MAX_HZ,
                     &encoded[0], &encoded_count[0]) != 0 ||
        modem_encode(&sent[1], MODEM_MIN_HZ, MODEM_MAX_HZ,
                     &encoded[1], &encoded_count[1]) != 0)
        goto done;
    stream_count = 23u + encoded_count[0] + 17u + encoded_count[1] + 41u;
    stream = calloc(stream_count, sizeof(*stream));
    decoder = modem_decoder_create(MODEM_MIN_HZ, MODEM_MAX_HZ);
    if (!stream || !decoder) {
        modem_decoder_destroy(decoder);
        goto done;
    }
    memcpy(stream + 23u, encoded[0], encoded_count[0] * sizeof(*stream));
    memcpy(stream + 23u + encoded_count[0] + 17u, encoded[1],
           encoded_count[1] * sizeof(*stream));
    while (position < stream_count) {
        size_t chunk = stream_count - position;
        int result;
        if (chunk > 257u)
            chunk = 257u;
        result = modem_decoder_feed(decoder, stream + position, chunk,
                                    &received);
        position += chunk;
        while (result == 1) {
            if (decoded >= 2u || received.type != sent[decoded].type ||
                received.session != sent[decoded].session ||
                received.seq != sent[decoded].seq)
                goto stream_done;
            ++decoded;
            result = modem_decoder_feed(decoder, NULL, 0, &received);
        }
        if (result < 0)
            goto stream_done;
    }
    status = decoded == 2u ? 0 : -1;
stream_done:
    modem_decoder_destroy(decoder);
done:
    free(stream);
    free(encoded[0]);
    free(encoded[1]);
    if (status != 0)
        fprintf(stderr, "modem streaming self-test failed\n");
    return status;
}

int modem_self_test(void)
{
    if (test_one_band(MODEM_MIN_HZ, MODEM_MAX_HZ) != 0 ||
        test_one_band(3500u, 8500u) != 0 ||
        test_clock_mismatch(0.998) != 0 ||
        test_clock_mismatch(1.002) != 0 || test_frame_stream() != 0)
        return -1;
    return 0;
}
