#include "modem.h"

#include "audio.h"

#include <complex.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define HEADER_N 16u
#define RAW_N (HEADER_N + MODEM_PAYLOAD_MAX + 4u)
#define CONTROL_RAW_N 13u
#define RAW_BITS (RAW_N * 8u)
#define CODE_STEPS (RAW_BITS + 6u)
#define CODE_BITS (CODE_STEPS * 2u)
#define PREAMBLE_UP 3u
#define PREAMBLE_DOWN 2u
#define PREFIX_SYMBOLS (PREAMBLE_UP + PREAMBLE_DOWN + 1u)
#define LEAD_N 240u
#define TAIL_N 240u
#define DETECT_LOOKAHEAD 128u
#define MIX_FILTER_TAPS 65u
#define OUTPUT_FILTER_TAPS 97u
#define RX_CAPACITY (AUDIO_SAMPLE_RATE * 12u)
#define CSS_SF 7u
#define CSS_M (1u << CSS_SF)
#define CSS_DATA_BITS 6u
#define SYNC_CONTROL 16u
#define SYNC_DATA_BASE 40u
#define SYNC_DATA_STEP 20u
#define DATA_PILOT_INTERVAL 24u
#define CONTROL_PILOT_INTERVAL 12u
#define PI 3.14159265358979323846

typedef struct {
    unsigned low_hz;
    unsigned high_hz;
    unsigned samples_per_chip;
    double chip_rate;
    double center_hz;
    double mix_filter[MIX_FILTER_TAPS];
} band_t;

struct modem_decoder {
    band_t band;
    enum modem_profile profile;
    float *rx;
    size_t length;
    size_t search_position;
    double peak_sync;
    unsigned candidates;
    unsigned rejected;
    unsigned timing_rejected;
    unsigned sync_rejected;
    unsigned pilot_rejected;
    unsigned payload_rejected;
    int pending;
    double pending_body_start;
    double pending_frequency_offset;
    enum modem_profile pending_profile;
    size_t pending_raw_length;
    size_t pending_needed;
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
        for (bit = 0; bit < 8u; ++bit)
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

size_t modem_profile_payload_limit(enum modem_profile profile)
{
    if (profile < MODEM_PROFILE_SAFE || profile >= MODEM_PROFILE_COUNT)
        return 0;
    return MODEM_PAYLOAD_MAX;
}

unsigned modem_profile_spreading_factor(enum modem_profile profile)
{
    if (profile < MODEM_PROFILE_SAFE || profile >= MODEM_PROFILE_COUNT)
        return 0;
    return CSS_SF;
}

static size_t data_raw_length(enum modem_profile profile)
{
    return HEADER_N + modem_profile_payload_limit(profile) + 4u;
}

static void design_lowpass(double *filter, unsigned taps, double cutoff_hz)
{
    unsigned i;
    int half = (int)taps / 2;
    double sum = 0.0;

    for (i = 0; i < taps; ++i) {
        int m = (int)i - half;
        double normalized = cutoff_hz / AUDIO_SAMPLE_RATE;
        double sinc = m == 0 ? 2.0 * normalized
                             : sin(2.0 * PI * normalized * m) / (PI * m);
        double window = 0.42 - 0.5 * cos(2.0 * PI * i / (taps - 1u)) +
                        0.08 * cos(4.0 * PI * i / (taps - 1u));
        filter[i] = sinc * window;
        sum += filter[i];
    }
    if (fabs(sum) > 1.0e-12)
        for (i = 0; i < taps; ++i)
            filter[i] /= sum;
}

static int make_band(unsigned low_hz, unsigned high_hz, band_t *band)
{
    unsigned requested_width;

    if (!band || low_hz < MODEM_MIN_HZ || high_hz > MODEM_MAX_HZ ||
        low_hz + MODEM_MIN_BANDWIDTH_HZ > high_hz)
        return -1;
    requested_width = high_hz - low_hz;
    band->samples_per_chip =
        (AUDIO_SAMPLE_RATE + requested_width - 1u) / requested_width;
    if (band->samples_per_chip < 2u)
        band->samples_per_chip = 2u;
    band->chip_rate = (double)AUDIO_SAMPLE_RATE / band->samples_per_chip;
    band->center_hz = (low_hz + high_hz) / 2.0;
    band->low_hz = low_hz;
    band->high_hz = high_hz;
    design_lowpass(band->mix_filter, MIX_FILTER_TAPS,
                   band->chip_rate * 0.58);
    return 0;
}

static size_t symbol_samples(const band_t *band, unsigned sf)
{
    return ((size_t)1u << sf) * band->samples_per_chip;
}

static size_t encoded_bits_for_raw(size_t raw_length)
{
    size_t steps;

    steps = raw_length * 8u + 6u;
    if (raw_length == CONTROL_RAW_N)
        return (steps / 3u) * 4u +
               (steps % 3u == 0u ? 0u
                                  : (steps % 3u == 1u ? 2u : 3u));
    return steps + (steps + 1u) / 2u;
}

static unsigned symbols_for_raw(size_t raw_length)
{
    size_t bits = encoded_bits_for_raw(raw_length);
    return (unsigned)((bits + CSS_DATA_BITS - 1u) / CSS_DATA_BITS);
}

static unsigned transmitted_symbols_for_raw(size_t raw_length)
{
    unsigned symbols = symbols_for_raw(raw_length);
    unsigned interval = raw_length == CONTROL_RAW_N
                            ? CONTROL_PILOT_INTERVAL
                            : DATA_PILOT_INTERVAL;

    symbols += (symbols + interval - 1u) / interval;
    return symbols;
}

static size_t frame_samples(const band_t *band, size_t raw_length,
                            enum modem_profile profile)
{
    size_t span = symbol_samples(band, CSS_SF);
    (void)profile;
    return LEAD_N +
           (PREFIX_SYMBOLS +
            (size_t)transmitted_symbols_for_raw(raw_length)) * span +
           TAIL_N;
}

static double css_phase(double chips, unsigned m)
{
    double wrapped = fmod(chips, m);
    if (wrapped < 0.0)
        wrapped += m;
    return PI * (wrapped * wrapped / m - wrapped);
}

static void emit_chirp(const band_t *band, unsigned value, int down,
                       size_t absolute_start, float *output)
{
    size_t count = symbol_samples(band, CSS_SF);
    double initial = css_phase(value, CSS_M);
    size_t i;

    for (i = 0; i < count; ++i) {
        double chips = (double)i / band->samples_per_chip + value;
        double base = css_phase(chips, CSS_M) - initial;
        double carrier = 2.0 * PI * band->center_hz *
                         (absolute_start + i) /
                         AUDIO_SAMPLE_RATE;
        output[i] = (float)(0.72 * cos(carrier + (down ? -base : base)));
    }
}

static void fft(double complex *x, unsigned count, int inverse)
{
    unsigned i, j, length;

    for (i = 1, j = 0; i < count; ++i) {
        unsigned bit = count >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j) {
            double complex temporary = x[i];
            x[i] = x[j];
            x[j] = temporary;
        }
    }
    for (length = 2; length <= count; length <<= 1) {
        double angle = (inverse ? 2.0 : -2.0) * PI / length;
        double complex step = cos(angle) + I * sin(angle);
        unsigned base;
        for (base = 0; base < count; base += length) {
            double complex rotation = 1.0;
            for (j = 0; j < length / 2u; ++j) {
                double complex a = x[base + j];
                double complex b = x[base + j + length / 2u] * rotation;
                x[base + j] = a + b;
                x[base + j + length / 2u] = a - b;
                rotation *= step;
            }
        }
    }
    if (inverse)
        for (i = 0; i < count; ++i)
            x[i] /= count;
}

static uint32_t gray_encode(uint32_t value)
{
    return value ^ (value >> 1);
}

static void serialize_frame(const modem_frame_t *frame, uint8_t raw[RAW_N])
{
    size_t content_length;
    uint32_t crc;

    memset(raw, 0, RAW_N);
    raw[0] = 0xa7;
    raw[1] = (uint8_t)(0x20u | (frame->type & 0x0fu));
    raw[2] = frame->flags;
    put_u32(raw + 3, frame->session);
    put_u16(raw + 7, frame->seq);
    put_u16(raw + 9, frame->ack);
    put_u16(raw + 11, frame->band_low);
    put_u16(raw + 13, frame->band_high);
    raw[15] = (uint8_t)frame->payload_len;
    memcpy(raw + HEADER_N, frame->payload, frame->payload_len);
    content_length = HEADER_N + frame->payload_len;
    crc = crc32(raw, content_length);
    put_u32(raw + content_length, crc);
}

static void serialize_control(const modem_frame_t *frame, uint8_t raw[RAW_N])
{
    uint8_t flags = frame->flags;
    uint16_t first, second;

    memset(raw, 0, RAW_N);
    raw[0] = 0xa7;
    raw[1] = (uint8_t)(0x20u | (frame->type & 0x0fu));
    if (frame->type == MODEM_PROFILE_SELECT) {
        first = frame->seq;
        second = frame->ack;
        if (frame->band_low)
            flags |= 0x80u;
    } else if (frame->type == MODEM_REQUEST ||
               frame->type == MODEM_RESPONSE) {
        first = frame->seq;
        second = frame->ack;
    } else {
        first = frame->band_low;
        second = frame->band_high;
    }
    raw[2] = flags;
    put_u32(raw + 3, frame->session);
    put_u16(raw + 7, first);
    put_u16(raw + 9, second);
    put_u16(raw + 11, (uint16_t)crc32(raw, 11u));
}

static int parse_frame(const uint8_t raw[RAW_N], size_t raw_length,
                       modem_frame_t *frame)
{
    uint16_t payload_length;

    if (raw_length == CONTROL_RAW_N) {
        if (raw[0] != 0xa7 || (raw[1] >> 4) != 2u ||
            (raw[1] & 0x0fu) < MODEM_HELLO ||
            (raw[1] & 0x0fu) > MODEM_PROFILE_SELECT ||
            get_u16(raw + 11) != (uint16_t)crc32(raw, 11u))
            return -1;
        memset(frame, 0, sizeof(*frame));
        frame->type = raw[1] & 0x0fu;
        frame->flags = raw[2];
        frame->session = get_u32(raw + 3);
        if (frame->type == MODEM_PROFILE_SELECT) {
            frame->seq = get_u16(raw + 7);
            frame->ack = get_u16(raw + 9);
            frame->band_low = (frame->flags & 0x80u) != 0u;
            frame->flags &= 0x7fu;
        } else if (frame->type == MODEM_REQUEST ||
                   frame->type == MODEM_RESPONSE) {
            frame->seq = get_u16(raw + 7);
            frame->ack = get_u16(raw + 9);
        } else {
            frame->band_low = get_u16(raw + 7);
            frame->band_high = get_u16(raw + 9);
        }
        return 0;
    }
    if (raw_length < CONTROL_RAW_N || raw[0] != 0xa7 ||
        (raw[1] >> 4) != 2u || (raw[1] & 0x0fu) < MODEM_HELLO ||
        (raw[1] & 0x0fu) > MODEM_PROFILE_SELECT)
        return -1;
    payload_length = raw[15];
    if (payload_length > MODEM_PAYLOAD_MAX ||
        payload_length > raw_length - HEADER_N - 4u ||
        HEADER_N + payload_length + 4u > raw_length ||
        get_u32(raw + HEADER_N + payload_length) !=
            crc32(raw, HEADER_N + payload_length))
        return -1;
    memset(frame, 0, sizeof(*frame));
    frame->type = raw[1] & 0x0fu;
    frame->flags = raw[2];
    frame->session = get_u32(raw + 3);
    frame->seq = get_u16(raw + 7);
    frame->ack = get_u16(raw + 9);
    frame->band_low = get_u16(raw + 11);
    frame->band_high = get_u16(raw + 13);
    frame->payload_len = payload_length;
    memcpy(frame->payload, raw + HEADER_N, payload_length);
    return 0;
}

static size_t convolutional_encode(const uint8_t *raw, size_t raw_length,
                                   uint8_t code[CODE_BITS])
{
    size_t raw_bits = raw_length * 8u;
    size_t steps = raw_bits + 6u;
    size_t position = 0;
    unsigned state = 0;
    size_t step;

    for (step = 0; step < steps; ++step) {
        unsigned input = step < raw_bits
                             ? (raw[step / 8u] >> (7u - step % 8u)) & 1u
                             : 0u;
        unsigned reg = ((state << 1) | input) & 0x7fu;
        uint8_t a = (uint8_t)parity(reg & 0x79u);
        uint8_t b = (uint8_t)parity(reg & 0x5bu);

        if (raw_length == CONTROL_RAW_N) {
            if (step % 3u != 2u)
                code[position++] = a;
            if (step % 3u != 1u)
                code[position++] = b;
        } else {
            code[position++] = a;
            if ((step & 1u) == 0u)
                code[position++] = b;
        }
        state = reg & 0x3fu;
    }
    return position;
}

static int convolutional_decode(const float llr[CODE_BITS], size_t raw_length,
                                uint8_t raw[RAW_N])
{
    float metric[64], next_metric[64];
    uint8_t *trace;
    size_t raw_bits = raw_length * 8u;
    size_t steps = raw_bits + 6u;
    size_t step, code_position = 0;
    unsigned state, input;

    trace = malloc(steps * 64u);
    if (!trace)
        return -1;
    for (state = 0; state < 64u; ++state)
        metric[state] = state == 0 ? 0.0f : 1.0e30f;
    for (step = 0; step < steps; ++step) {
        float llr_a, llr_b;

        if (raw_length == CONTROL_RAW_N) {
            llr_a = step % 3u != 2u ? llr[code_position++] : 0.0f;
            llr_b = step % 3u != 1u ? llr[code_position++] : 0.0f;
        } else {
            llr_a = llr[code_position++];
            llr_b = (step & 1u) == 0u ? llr[code_position++] : 0.0f;
        }
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
                float branch = (a ? -llr_a : llr_a) +
                               (b ? -llr_b : llr_b);
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
    memset(raw, 0, raw_length);
    state = 0;
    for (step = steps; step-- > 0;) {
        uint8_t choice = trace[step * 64u + state];
        input = choice >> 6;
        if (step < raw_bits)
            raw[step / 8u] |= (uint8_t)(input << (7u - step % 8u));
        state = choice & 63u;
    }
    free(trace);
    return 0;
}

static unsigned sync_value(enum modem_profile profile, int control)
{
    return control ? SYNC_CONTROL
                   : SYNC_DATA_BASE + (unsigned)profile * SYNC_DATA_STEP;
}

static void emit_bits_body(const band_t *band, const uint8_t *bits,
                           size_t bit_count, unsigned pilot_interval,
                           size_t absolute_start,
                           float *output)
{
    unsigned symbol_count =
        (unsigned)((bit_count + CSS_DATA_BITS - 1u) / CSS_DATA_BITS);
    size_t span = symbol_samples(band, CSS_SF);
    unsigned symbol, bit, transmitted = 0;

    for (symbol = 0; symbol < symbol_count; ++symbol) {
        uint32_t word = 0;

        if (symbol % pilot_interval == 0u) {
            emit_chirp(band, 0u, 0, absolute_start + transmitted * span,
                       output + transmitted * span);
            ++transmitted;
        }
        for (bit = 0; bit < CSS_DATA_BITS; ++bit) {
            size_t index = (size_t)bit * symbol_count + symbol;
            word = (word << 1) | (index < bit_count ? bits[index] : 0u);
        }
        emit_chirp(band, gray_encode(word) << 1u, 0,
                   absolute_start + transmitted * span,
                   output + transmitted * span);
        ++transmitted;
    }
}

static int apply_bandpass(float *samples, size_t count, unsigned low_hz,
                          unsigned high_hz)
{
    double filter[OUTPUT_FILTER_TAPS];
    float *filtered;
    unsigned tap;
    int half = (int)OUTPUT_FILTER_TAPS / 2;
    double response = 0.0;
    size_t i;

    for (tap = 0; tap < OUTPUT_FILTER_TAPS; ++tap) {
        int m = (int)tap - half;
        double high = (double)high_hz / AUDIO_SAMPLE_RATE;
        double low = (double)low_hz / AUDIO_SAMPLE_RATE;
        double ideal_high = m == 0 ? 2.0 * high
                                   : sin(2.0 * PI * high * m) / (PI * m);
        double ideal_low = m == 0 ? 2.0 * low
                                  : sin(2.0 * PI * low * m) / (PI * m);
        double window = 0.42 -
                        0.5 * cos(2.0 * PI * tap /
                                  (OUTPUT_FILTER_TAPS - 1u)) +
                        0.08 * cos(4.0 * PI * tap /
                                   (OUTPUT_FILTER_TAPS - 1u));
        filter[tap] = (ideal_high - ideal_low) * window;
        response += filter[tap] *
                    cos(2.0 * PI * ((low_hz + high_hz) / 2.0) * m /
                        AUDIO_SAMPLE_RATE);
    }
    if (fabs(response) > 1.0e-9)
        for (tap = 0; tap < OUTPUT_FILTER_TAPS; ++tap)
            filter[tap] /= response;
    filtered = calloc(count, sizeof(*filtered));
    if (!filtered)
        return -1;
    for (i = 0; i < count; ++i) {
        double value = 0.0;
        for (tap = 0; tap < OUTPUT_FILTER_TAPS; ++tap) {
            long source = (long)i + (long)tap - half;
            if (source >= 0 && (size_t)source < count)
                value += samples[source] * filter[tap];
        }
        filtered[i] = (float)value;
    }
    memcpy(samples, filtered, count * sizeof(*samples));
    free(filtered);
    return 0;
}

int modem_encode(const modem_frame_t *frame, unsigned low_hz, unsigned high_hz,
                 enum modem_profile profile, float **samples, size_t *count)
{
    band_t band;
    uint8_t raw[RAW_N], code[CODE_BITS];
    float *output;
    size_t raw_length, bit_count, total, position, span, active_end;
    unsigned preamble;
    int control;
    double peak = 0.0;

    if (!frame || !samples || !count || profile < MODEM_PROFILE_SAFE ||
        profile >= MODEM_PROFILE_COUNT ||
        frame->payload_len > MODEM_PAYLOAD_MAX ||
        (frame->payload_len != 0 &&
         frame->payload_len > modem_profile_payload_limit(profile)) ||
        make_band(low_hz, high_hz, &band) != 0)
        return -1;
    control = frame->payload_len == 0;
    raw_length = control ? CONTROL_RAW_N : data_raw_length(profile);
    if (control)
        serialize_control(frame, raw);
    else
        serialize_frame(frame, raw);
    bit_count = convolutional_encode(raw, raw_length, code);
    total = frame_samples(&band, raw_length, profile);
    output = calloc(total, sizeof(*output));
    if (!output)
        return -1;
    span = symbol_samples(&band, CSS_SF);
    position = LEAD_N;
    for (preamble = 0; preamble < PREAMBLE_UP; ++preamble) {
        emit_chirp(&band, 0u, 0, position, output + position);
        position += span;
    }
    for (preamble = 0; preamble < PREAMBLE_DOWN; ++preamble) {
        emit_chirp(&band, 0u, 1, position, output + position);
        position += span;
    }
    emit_chirp(&band, sync_value(profile, control), 0, position,
               output + position);
    position += span;
    emit_bits_body(&band, code, bit_count,
                   control ? CONTROL_PILOT_INTERVAL : DATA_PILOT_INTERVAL,
                   position,
                   output + position);

    /* Fade only the burst edges. Per-symbol fades create timing ambiguity and
       the discontinuous oscillator resets that caused audible crackle. */
    active_end = total - TAIL_N;
    for (preamble = 0; preamble < 120u; ++preamble) {
        double gain = sin(0.5 * PI * (preamble + 1u) / 120.0);
        gain *= gain;
        output[LEAD_N + preamble] *= (float)gain;
        output[active_end - 1u - preamble] *= (float)gain;
    }

    if (apply_bandpass(output, total, low_hz, high_hz) != 0) {
        free(output);
        return -1;
    }
    for (position = 0; position < total; ++position) {
        double absolute = fabs(output[position]);
        if (absolute > peak)
            peak = absolute;
    }
    if (peak > 0.0) {
        float scale = (float)(0.50 / peak);
        for (position = 0; position < total; ++position)
            output[position] *= scale;
    }
    *samples = output;
    *count = total;
    return 0;
}

size_t modem_burst_samples(unsigned low_hz, unsigned high_hz,
                           enum modem_profile profile)
{
    band_t band;
    if (profile < MODEM_PROFILE_SAFE || profile >= MODEM_PROFILE_COUNT ||
        make_band(low_hz, high_hz, &band) != 0)
        return 0;
    return frame_samples(&band, data_raw_length(profile), profile);
}

size_t modem_control_burst_samples(unsigned low_hz, unsigned high_hz)
{
    band_t band;

    if (make_band(low_hz, high_hz, &band) != 0)
        return 0;
    return frame_samples(&band, CONTROL_RAW_N, MODEM_PROFILE_SAFE);
}

void modem_free_samples(float *samples)
{
    free(samples);
}

static float interpolate(const float *samples, size_t length, double position)
{
    size_t index;
    double fraction;

    if (position <= 0.0)
        return samples[0];
    index = (size_t)position;
    if (index + 1u >= length)
        return samples[length - 1u];
    fraction = position - index;
    return (float)(samples[index] * (1.0 - fraction) +
                   samples[index + 1u] * fraction);
}

static int css_spectrum(const modem_decoder_t *decoder, double start,
                        int down, double power[CSS_M],
                        unsigned *peak, double *quality)
{
    size_t span = symbol_samples(&decoder->band, CSS_SF);
    double complex bins[CSS_M];
    double total_power = 0.0, peak_power = 0.0;
    unsigned chip, k;
    int half = (int)MIX_FILTER_TAPS / 2;

    if (start < 0.0 || start + span + half + 2.0 >= decoder->length)
        return -1;
    for (chip = 0; chip < CSS_M; ++chip) {
        unsigned chip_offset = decoder->band.samples_per_chip / 2u;
        double center = start + chip * decoder->band.samples_per_chip +
                        chip_offset;
        double complex analytic = 0.0;
        unsigned tap;
        for (tap = 0; tap < MIX_FILTER_TAPS; ++tap) {
            int displacement = (int)tap - half;
            double position = center + displacement;
            double carrier = 2.0 * PI * decoder->band.center_hz * position /
                             AUDIO_SAMPLE_RATE;
            double sample = interpolate(decoder->rx, decoder->length,
                                        position);
            analytic += decoder->band.mix_filter[tap] * sample * 2.0 *
                        (cos(carrier) - I * sin(carrier));
        }
        {
            double chips = chip +
                           (double)chip_offset /
                               decoder->band.samples_per_chip;
            double phase = css_phase(chips, CSS_M);
            if (down)
                phase = -phase;
            bins[chip] = analytic * (cos(phase) - I * sin(phase));
        }
    }
    fft(bins, CSS_M, 0);
    *peak = 0;
    for (k = 0; k < CSS_M; ++k) {
        power[k] = creal(bins[k]) * creal(bins[k]) +
                   cimag(bins[k]) * cimag(bins[k]);
        total_power += power[k];
        if (power[k] > peak_power) {
            peak_power = power[k];
            *peak = k;
        }
    }
    *quality = peak_power /
               (total_power / CSS_M > 1.0e-12
                    ? total_power / CSS_M
                    : 1.0e-12);
    return 0;
}

static double fractional_peak(const double power[CSS_M], unsigned peak)
{
    double left = log(power[(peak + CSS_M - 1u) & (CSS_M - 1u)] + 1.0e-30);
    double center = log(power[peak] + 1.0e-30);
    double right = log(power[(peak + 1u) & (CSS_M - 1u)] + 1.0e-30);
    double denominator = left - 2.0 * center + right;
    double offset = fabs(denominator) > 1.0e-12
                        ? 0.5 * (left - right) / denominator
                        : 0.0;

    if (offset < -0.5)
        offset = -0.5;
    if (offset > 0.5)
        offset = 0.5;
    return peak + offset;
}

static double wrap_signed(double value)
{
    while (value > (double)CSS_M / 2.0)
        value -= CSS_M;
    while (value <= -(double)CSS_M / 2.0)
        value += CSS_M;
    return value;
}

static double circular_error(double observed, double expected)
{
    return fabs(wrap_signed(observed - expected));
}

/* Returns 0 for a frame, 1 for noise, -1 for an upchirp without the expected
   downchirp transition, and -2 for a preamble with an unknown sync word. */
static int acquire_frame(const modem_decoder_t *decoder, size_t coarse,
                         double *body_start, double *frequency_offset,
                         enum modem_profile *profile, size_t *raw_length,
                         double *score)
{
    double up_power[CSS_M], up2_power[CSS_M];
    double down_power[CSS_M], sync_power[CSS_M];
    double up_quality, up2_quality, down_quality = 0.0;
    double best_sync_quality = 0.0;
    double up_bin, down_bin, delta, cfo, down_start;
    size_t span = symbol_samples(&decoder->band, CSS_SF);
    unsigned up_peak, up2_peak, down_peak = 0, sync_peak;
    unsigned step, sync_step;
    int found_down = 0;

    if (css_spectrum(decoder, coarse, 0, up_power, &up_peak,
                     &up_quality) != 0 || up_quality < 8.0)
        return 1;
    if (css_spectrum(decoder, coarse + span, 0, up2_power, &up2_peak,
                     &up2_quality) != 0 || up2_quality < 8.0 ||
        circular_error(fractional_peak(up_power, up_peak),
                       fractional_peak(up2_power, up2_peak)) > 2.0)
        return 1;
    *score = up_quality < up2_quality ? up_quality : up2_quality;
    for (step = 1u; step <= PREAMBLE_UP; ++step) {
        if (css_spectrum(decoder, coarse + (size_t)step * span, 1,
                         down_power, &down_peak, &down_quality) == 0 &&
            down_quality >= 8.0) {
            found_down = 1;
            break;
        }
    }
    if (!found_down)
        return -1;
    if (down_quality < *score)
        *score = down_quality;

    up_bin = fractional_peak(up_power, up_peak);
    up_bin += 0.5 * wrap_signed(fractional_peak(up2_power, up2_peak) -
                                up_bin);
    down_bin = fractional_peak(down_power, down_peak);
    delta = 0.5 * wrap_signed(up_bin - down_bin);
    cfo = wrap_signed(up_bin - delta);
    if (fabs(cfo) > 12.0)
        return -1;
    down_start = coarse + (double)step * span -
                 delta * decoder->band.samples_per_chip;

    for (sync_step = 1u; sync_step <= PREAMBLE_DOWN; ++sync_step) {
        double candidate_start = down_start + (double)sync_step * span;
        double quality, adjusted;
        enum modem_profile candidate;

        if (css_spectrum(decoder, candidate_start, 0, sync_power,
                         &sync_peak, &quality) != 0)
            continue;
        if (quality < 6.0)
            continue;
        adjusted = fractional_peak(sync_power, sync_peak) - cfo;
        if (circular_error(adjusted, SYNC_CONTROL) <= 3.0 &&
            quality > best_sync_quality) {
            *profile = MODEM_PROFILE_SAFE;
            *raw_length = CONTROL_RAW_N;
            *body_start = candidate_start + span;
            best_sync_quality = quality;
        }
        for (candidate = MODEM_PROFILE_SAFE;
             candidate < MODEM_PROFILE_COUNT;
             candidate = (enum modem_profile)(candidate + 1)) {
            if (candidate == decoder->profile &&
                circular_error(adjusted, sync_value(candidate, 0)) <= 3.0 &&
                quality > best_sync_quality) {
                *profile = candidate;
                *raw_length = data_raw_length(candidate);
                *body_start = candidate_start + span;
                best_sync_quality = quality;
            }
        }
    }
    if (best_sync_quality == 0.0)
        return -2;
    *frequency_offset = cfo;
    if (best_sync_quality < *score)
        *score = best_sync_quality;
    return 0;
}

static void spectrum_llrs(const double power[CSS_M], double frequency_offset,
                          float out[CSS_DATA_BITS])
{
    double best_zero[CSS_DATA_BITS] = {0};
    double best_one[CSS_DATA_BITS] = {0};
    unsigned word, bit;

    for (word = 0; word < (1u << CSS_DATA_BITS); ++word) {
        long center = lround((gray_encode(word) << 1u) + frequency_offset);
        unsigned candidate = (unsigned)center & (CSS_M - 1u);
        double candidate_power = power[candidate];
        double magnitude;

        magnitude = sqrt(candidate_power);
        for (bit = 0; bit < CSS_DATA_BITS; ++bit) {
            unsigned mask = 1u << (CSS_DATA_BITS - 1u - bit);
            double *best = (word & mask) ? &best_one[bit] : &best_zero[bit];
            if (magnitude > *best)
                *best = magnitude;
        }
    }
    for (bit = 0; bit < CSS_DATA_BITS; ++bit) {
        double scale = best_one[bit] + best_zero[bit] + 1.0e-12;
        out[bit] = (float)(6.0 * (best_one[bit] - best_zero[bit]) / scale);
    }
}

static int decode_pending(modem_decoder_t *decoder, modem_frame_t *frame)
{
    uint8_t raw[RAW_N];
    float llr[CODE_BITS];
    size_t bit_count = encoded_bits_for_raw(decoder->pending_raw_length);
    unsigned symbol_count =
        (unsigned)((bit_count + CSS_DATA_BITS - 1u) / CSS_DATA_BITS);
    unsigned pilot_interval = decoder->pending_raw_length == CONTROL_RAW_N
                                  ? CONTROL_PILOT_INTERVAL
                                  : DATA_PILOT_INTERVAL;
    size_t span = symbol_samples(&decoder->band, CSS_SF);
    double cursor = decoder->pending_body_start;
    unsigned symbol, bit;
    memset(llr, 0, sizeof(llr));
    for (symbol = 0; symbol < symbol_count; ++symbol) {
        double power[CSS_M], quality;
        float bits[CSS_DATA_BITS];
        double start;
        unsigned peak;

        if (symbol % pilot_interval == 0u) {
            double timing_error;

            if (css_spectrum(decoder, cursor, 0, power, &peak, &quality) != 0)
                return 1;
            timing_error = wrap_signed(fractional_peak(power, peak) -
                                       decoder->pending_frequency_offset);
            if (quality < 6.0 || fabs(timing_error) > 8.0) {
                ++decoder->pilot_rejected;
                return -1;
            }
            cursor -= timing_error * decoder->band.samples_per_chip;
            cursor += span;
        }
        start = cursor;
        if (css_spectrum(decoder, start, 0, power, &peak, &quality) != 0)
            return 1;
        if (quality < 2.0) {
            ++decoder->payload_rejected;
            return -1;
        }
        (void)peak;
        spectrum_llrs(power, decoder->pending_frequency_offset, bits);
        for (bit = 0; bit < CSS_DATA_BITS; ++bit) {
            size_t index = (size_t)bit * symbol_count + symbol;
            if (index < bit_count)
                llr[index] = bits[bit];
        }
        cursor = start + span;
    }
    if (convolutional_decode(llr, decoder->pending_raw_length, raw) != 0) {
        ++decoder->payload_rejected;
        return -1;
    }
    if (parse_frame(raw, decoder->pending_raw_length, frame) != 0) {
        ++decoder->payload_rejected;
        return -1;
    }
    return 0;
}

modem_decoder_t *modem_decoder_create(unsigned low_hz, unsigned high_hz)
{
    modem_decoder_t *decoder = calloc(1, sizeof(*decoder));

    if (!decoder)
        return NULL;
    decoder->profile = MODEM_PROFILE_SAFE;
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
    decoder->pending = 0;
    return 0;
}

int modem_decoder_set_profile(modem_decoder_t *decoder,
                              enum modem_profile profile)
{
    if (!decoder || profile < MODEM_PROFILE_SAFE ||
        profile >= MODEM_PROFILE_COUNT)
        return -1;
    decoder->profile = profile;
    decoder->length = 0;
    decoder->search_position = 0;
    decoder->pending = 0;
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
    activity->timing_rejected = decoder->timing_rejected;
    activity->sync_rejected = decoder->sync_rejected;
    activity->pilot_rejected = decoder->pilot_rejected;
    activity->payload_rejected = decoder->payload_rejected;
    decoder->peak_sync = 0.0;
    decoder->candidates = 0;
    decoder->rejected = 0;
    decoder->timing_rejected = 0;
    decoder->sync_rejected = 0;
    decoder->pilot_rejected = 0;
    decoder->payload_rejected = 0;
}

static void discard_samples(modem_decoder_t *decoder, size_t count)
{
    if (count >= decoder->length) {
        decoder->length = 0;
        decoder->search_position = 0;
        decoder->pending = 0;
        return;
    }
    memmove(decoder->rx, decoder->rx + count,
            (decoder->length - count) * sizeof(*decoder->rx));
    decoder->length -= count;
    decoder->search_position = decoder->search_position > count
                                   ? decoder->search_position - count
                                   : 0;
    if (decoder->pending) {
        if (decoder->pending_body_start >= count)
            decoder->pending_body_start -= count;
        else
            decoder->pending = 0;
    }
}

int modem_decoder_feed(modem_decoder_t *decoder, const float *samples,
                       size_t count, modem_frame_t *frame)
{
    size_t span;
    size_t prefix;
    unsigned scan_step;

    if (!decoder || !frame || (count && !samples))
        return -1;
    span = symbol_samples(&decoder->band, CSS_SF);
    prefix = PREFIX_SYMBOLS * span;
    scan_step = (unsigned)(span / 8u);
    if (count > RX_CAPACITY) {
        samples += count - RX_CAPACITY;
        count = RX_CAPACITY;
        decoder->length = 0;
        decoder->search_position = 0;
        decoder->pending = 0;
    } else if (decoder->length + count > RX_CAPACITY) {
        discard_samples(decoder, decoder->length + count - RX_CAPACITY);
    }
    if (count) {
        memcpy(decoder->rx + decoder->length, samples,
               count * sizeof(*samples));
        decoder->length += count;
    }

    if (decoder->pending) {
        int result;
        size_t discard;
        if (decoder->length < decoder->pending_needed)
            return 0;
        result = decode_pending(decoder, frame);
        if (result == 1) {
            decoder->pending_needed = decoder->length + DETECT_LOOKAHEAD;
            return 0;
        }
        discard = decoder->pending_needed > DETECT_LOOKAHEAD
                      ? decoder->pending_needed - DETECT_LOOKAHEAD
                      : decoder->pending_needed;
        decoder->pending = 0;
        discard_samples(decoder, discard);
        if (result == 0)
            return 1;
        ++decoder->rejected;
    }

    while (decoder->length >= prefix + DETECT_LOOKAHEAD) {
        size_t max_start = decoder->length - prefix - DETECT_LOOKAHEAD;
        size_t position;
        int found = 0;

        for (position = decoder->search_position; position <= max_start;
             position += scan_step) {
            double body_start = 0.0, frequency_offset = 0.0, score = 0.0;
            enum modem_profile profile = MODEM_PROFILE_SAFE;
            size_t raw_length = 0;
            int result = acquire_frame(decoder, position, &body_start,
                                       &frequency_offset, &profile,
                                       &raw_length, &score);
            double reported = score / (score + 8.0);

            if (reported > decoder->peak_sync)
                decoder->peak_sync = reported;
            if (result == 1)
                continue;
            ++decoder->candidates;
            if (result != 0) {
                if (result == -1)
                    ++decoder->timing_rejected;
                else
                    ++decoder->sync_rejected;
                ++decoder->rejected;
                continue;
            }
            decoder->pending = 1;
            decoder->pending_body_start = body_start;
            decoder->pending_frequency_offset = frequency_offset;
            decoder->pending_profile = profile;
            decoder->pending_raw_length = raw_length;
            decoder->pending_needed =
                (size_t)ceil(body_start +
                             transmitted_symbols_for_raw(raw_length) * span) +
                MIX_FILTER_TAPS / 2u + 2u;
            found = 1;
            break;
        }
        decoder->search_position = max_start + scan_step;
        if (!found)
            break;
        if (decoder->length < decoder->pending_needed)
            return 0;
        return modem_decoder_feed(decoder, NULL, 0, frame);
    }
    if (decoder->length > RX_CAPACITY - AUDIO_SAMPLE_RATE)
        discard_samples(decoder, decoder->length / 2u);
    return 0;
}

static uint32_t test_random(uint32_t *state)
{
    *state = *state * 1664525u + 1013904223u;
    return *state;
}

static int test_control_wire(void)
{
    static const uint8_t types[] = {
        MODEM_HELLO, MODEM_OFFER, MODEM_CONFIRM, MODEM_READY,
        MODEM_REQUEST, MODEM_RESPONSE, MODEM_PROFILE_SELECT};
    modem_frame_t sent, received;
    uint8_t raw[RAW_N];
    size_t i;

    for (i = 0; i < sizeof(types) / sizeof(types[0]); ++i) {
        memset(&sent, 0, sizeof(sent));
        sent.type = types[i];
        sent.flags = sent.type == MODEM_PROFILE_SELECT ? 3u : 1u;
        sent.session = 0x1234abcdu + (uint32_t)i;
        sent.seq = (uint16_t)(400u + i);
        sent.ack = (uint16_t)(300u + i);
        sent.band_low = sent.type == MODEM_PROFILE_SELECT
                            ? 1u : MODEM_MIN_HZ;
        sent.band_high = MODEM_MAX_HZ;
        serialize_control(&sent, raw);
        if (parse_frame(raw, CONTROL_RAW_N, &received) != 0 ||
            received.type != sent.type || received.flags != sent.flags ||
            received.session != sent.session)
            goto failure;
        if (sent.type == MODEM_REQUEST || sent.type == MODEM_RESPONSE ||
            sent.type == MODEM_PROFILE_SELECT) {
            if (received.seq != sent.seq || received.ack != sent.ack)
                goto failure;
            if (sent.type == MODEM_PROFILE_SELECT &&
                received.band_low != sent.band_low)
                goto failure;
        } else if (received.band_low != sent.band_low ||
                   received.band_high != sent.band_high) {
            goto failure;
        }
    }
    raw[5] ^= 0x40u;
    if (parse_frame(raw, CONTROL_RAW_N, &received) == 0)
        goto failure;
    return 0;

failure:
    fprintf(stderr, "compact control wire self-test failed\n");
    return -1;
}

static int test_css_symbols(void)
{
    band_t band;
    modem_decoder_t *decoder = NULL;
    double complex baseband[CSS_M];
    double power[CSS_M], quality;
    float *audio = NULL;
    size_t span, start = 137u, count;
    unsigned value, chip, bin, peak;
    int failed = -1;

    if (make_band(MODEM_MIN_HZ, MODEM_MAX_HZ, &band) != 0)
        return -1;

    /* First prove the CSS algebra without the real audio carrier. */
    for (value = 0; value < CSS_M; value += 2u) {
        double peak_power = 0.0;
        unsigned best = 0;

        for (chip = 0; chip < CSS_M; ++chip) {
            double phase = css_phase(chip + value, CSS_M) -
                           css_phase(value, CSS_M) -
                           css_phase(chip, CSS_M);
            baseband[chip] = cos(phase) + I * sin(phase);
        }
        fft(baseband, CSS_M, 0);
        for (bin = 0; bin < CSS_M; ++bin) {
            double magnitude = creal(baseband[bin]) * creal(baseband[bin]) +
                               cimag(baseband[bin]) * cimag(baseband[bin]);
            if (magnitude > peak_power) {
                peak_power = magnitude;
                best = bin;
            }
        }
        if (best != value) {
            fprintf(stderr,
                    "baseband CSS symbol test failed: sent bin %u, got %u\n",
                    value, best);
            return -1;
        }
    }

    /* Then prove every symbol through the real carrier and output filter. */
    span = symbol_samples(&band, CSS_SF);
    count = start + span + MIX_FILTER_TAPS + 128u;
    audio = calloc(count, sizeof(*audio));
    decoder = modem_decoder_create(MODEM_MIN_HZ, MODEM_MAX_HZ);
    if (!audio || !decoder)
        goto done;
    for (value = 0; value < CSS_M; value += 2u) {
        double best_power = 0.0;
        unsigned best = 0;

        peak = 0;
        quality = 0.0;
        memset(audio, 0, count * sizeof(*audio));
        emit_chirp(&band, value, 0, start, audio + start);
        if (apply_bandpass(audio, count, MODEM_MIN_HZ, MODEM_MAX_HZ) != 0)
            goto done;
        memcpy(decoder->rx, audio, count * sizeof(*audio));
        decoder->length = count;
        if (css_spectrum(decoder, start, 0, power, &peak, &quality) != 0 ||
            quality < 6.0)
            goto symbol_failure;
        for (bin = 0; bin < CSS_M; bin += 2u) {
            if (power[bin] > best_power) {
                best_power = power[bin];
                best = bin;
            }
        }
        if (best != value) {
symbol_failure:
            fprintf(stderr,
                    "passband CSS symbol test failed: sent bin %u, got %u "
                    "(raw peak %u, quality %.1f)\n",
                    value, best, peak, quality);
            goto done;
        }
    }
    failed = 0;
done:
    free(audio);
    modem_decoder_destroy(decoder);
    return failed;
}

static int test_css_acquisition(void)
{
    band_t band;
    modem_decoder_t *decoder = NULL;
    float *audio = NULL;
    size_t span, start = 311u, position, count, coarse;
    double expected_body, body, frequency, score;
    enum modem_profile profile;
    size_t raw_length;
    unsigned preamble;
    int result = -1;

    if (make_band(MODEM_MIN_HZ, MODEM_MAX_HZ, &band) != 0)
        return -1;
    span = symbol_samples(&band, CSS_SF);
    count = start + PREFIX_SYMBOLS * span + MIX_FILTER_TAPS + 256u;
    audio = calloc(count, sizeof(*audio));
    decoder = modem_decoder_create(MODEM_MIN_HZ, MODEM_MAX_HZ);
    if (!audio || !decoder)
        goto done;

    position = start;
    for (preamble = 0; preamble < PREAMBLE_UP; ++preamble) {
        emit_chirp(&band, 0u, 0, position, audio + position);
        position += span;
    }
    for (preamble = 0; preamble < PREAMBLE_DOWN; ++preamble) {
        emit_chirp(&band, 0u, 1, position, audio + position);
        position += span;
    }
    emit_chirp(&band, SYNC_CONTROL, 0, position, audio + position);
    if (apply_bandpass(audio, count, MODEM_MIN_HZ, MODEM_MAX_HZ) != 0)
        goto done;
    memcpy(decoder->rx, audio, count * sizeof(*audio));
    decoder->length = count;
    expected_body = start + PREFIX_SYMBOLS * span;

    for (coarse = 0; coarse < start + span; coarse += span / 8u) {
        int acquired = acquire_frame(decoder, coarse, &body, &frequency,
                                     &profile, &raw_length, &score);
        if (acquired == 0) {
            if (raw_length != CONTROL_RAW_N ||
                fabs(body - expected_body) > 2.0 ||
                fabs(frequency) > 0.5 || score < 6.0) {
                fprintf(stderr,
                        "CSS acquisition test failed: body %.2f/%.2f, "
                        "frequency %.2f, raw %zu\n",
                        body, expected_body, frequency, raw_length);
                goto done;
            }
            result = 0;
            break;
        }
    }
    if (result != 0)
        fprintf(stderr, "CSS acquisition test found no frame\n");
done:
    free(audio);
    modem_decoder_destroy(decoder);
    return result;
}

static int test_channel(unsigned low, unsigned high,
                        enum modem_profile profile, double ratio,
                        int control)
{
    modem_frame_t sent, received;
    modem_decoder_t *decoder = NULL;
    float *encoded = NULL, *channel = NULL;
    size_t encoded_count = 0, resampled_count, channel_count, i, position;
    uint32_t random = 7;
    int got = 0;

    memset(&sent, 0, sizeof(sent));
    sent.type = control ? MODEM_HELLO : MODEM_REQUEST;
    sent.flags = control ? 0 : 1;
    sent.session = 0x1234abcdu;
    sent.seq = control ? 0 : 417;
    sent.band_low = (uint16_t)low;
    sent.band_high = (uint16_t)high;
    sent.payload_len =
        control ? 0 : (uint16_t)modem_profile_payload_limit(profile);
    for (i = 0; i < sent.payload_len; ++i)
        sent.payload[i] = (uint8_t)test_random(&random);
    if (modem_encode(&sent, low, high, profile, &encoded, &encoded_count) != 0)
        goto done;
    resampled_count = (size_t)(encoded_count * ratio);
    channel_count = 137u + resampled_count + 911u + 1024u;
    channel = calloc(channel_count, sizeof(*channel));
    decoder = modem_decoder_create(low, high);
    if (!channel || !decoder ||
        modem_decoder_set_profile(decoder, profile) != 0)
        goto done;
    for (i = 0; i < channel_count; ++i) {
        double noise = ((int)(test_random(&random) >> 16) - 32768) / 32768.0;
        channel[i] = (float)(noise * 0.015);
    }
    for (i = 0; i < resampled_count; ++i) {
        double source = i / ratio;
        size_t index = (size_t)source;
        double fraction = source - index;
        float sample = index + 1u < encoded_count
                           ? (float)(encoded[index] * (1.0 - fraction) +
                                     encoded[index + 1u] * fraction)
                           : 0.0f;
        channel[137u + i] += sample * 0.34f;
        channel[137u + i + 31u] += sample * 0.08f;
        channel[137u + i + 347u] += sample * 0.11f;
        channel[137u + i + 911u] -= sample * 0.06f;
    }
    for (position = 0; position < channel_count && !got;) {
        size_t chunk = channel_count - position;
        int result;
        if (chunk > 997u)
            chunk = 997u;
        result = modem_decoder_feed(decoder, channel + position, chunk,
                                    &received);
        if (result < 0)
            break;
        got = result;
        position += chunk;
    }
    if (got && received.type == sent.type &&
        received.flags == sent.flags && received.session == sent.session &&
        received.seq == sent.seq && received.band_low == sent.band_low &&
        received.band_high == sent.band_high &&
        received.payload_len == sent.payload_len &&
        memcmp(received.payload, sent.payload, sent.payload_len) == 0) {
        free(channel);
        modem_free_samples(encoded);
        modem_decoder_destroy(decoder);
        return 0;
    }
done:
    if (decoder) {
        modem_activity_t activity;
        modem_decoder_take_activity(decoder, &activity);
        fprintf(stderr,
                "  decoder: got=%d, sync=%.3f, candidates=%u, rejected=%u "
                "(timing=%u, sync=%u, payload=%u)\n",
                got, activity.peak_sync, activity.candidates,
                activity.rejected, activity.timing_rejected,
                activity.sync_rejected, activity.payload_rejected);
    }
    free(channel);
    modem_free_samples(encoded);
    modem_decoder_destroy(decoder);
    fprintf(stderr,
            "CSS %s self-test failed for SF%u, %u-%u Hz at %.0f ppm\n",
            control ? "control" : "data",
            modem_profile_spreading_factor(profile), low, high,
            (ratio - 1.0) * 1000000.0);
    return -1;
}

int modem_self_test(void)
{
    enum modem_profile profile;
    uint8_t raw[RAW_N], decoded[RAW_N], code[CODE_BITS];
    float llr[CODE_BITS];
    size_t code_bits, i, control_samples, data_samples;
    double useful_rate;

    if (test_css_symbols() != 0 || test_css_acquisition() != 0 ||
        test_control_wire() != 0)
        return -1;

    for (i = 0; i < RAW_N; ++i)
        raw[i] = (uint8_t)(i * 37u + 11u);
    code_bits = convolutional_encode(raw, RAW_N, code);
    for (i = 0; i < code_bits; ++i)
        llr[i] = code[i] ? 6.0f : -6.0f;
    if (convolutional_decode(llr, RAW_N, decoded) != 0 ||
        memcmp(raw, decoded, RAW_N) != 0) {
        fprintf(stderr, "convolutional codec self-test failed\n");
        return -1;
    }

    for (i = 0; i < CONTROL_RAW_N; ++i)
        raw[i] = (uint8_t)(i * 53u + 7u);
    code_bits = convolutional_encode(raw, CONTROL_RAW_N, code);
    for (i = 0; i < code_bits; ++i)
        llr[i] = code[i] ? 6.0f : -6.0f;
    llr[17] = -llr[17];
    llr[71] = -llr[71];
    llr[129] = -llr[129];
    if (convolutional_decode(llr, CONTROL_RAW_N, decoded) != 0 ||
        memcmp(raw, decoded, CONTROL_RAW_N) != 0) {
        fprintf(stderr, "control FEC corruption self-test failed\n");
        return -1;
    }

    control_samples = modem_control_burst_samples(MODEM_MIN_HZ,
                                                  MODEM_MAX_HZ);
    data_samples = modem_burst_samples(MODEM_MIN_HZ, MODEM_MAX_HZ,
                                       MODEM_PROFILE_SAFE);
    useful_rate = (MODEM_PAYLOAD_MAX - 8u) * 8.0 * AUDIO_SAMPLE_RATE /
                  data_samples;
    if (control_samples > AUDIO_SAMPLE_RATE * 47u / 100u ||
        useful_rate < 200.0) {
        fprintf(stderr,
                "modem rate self-test failed: %.3f s control burst, "
                "%.1f useful bit/s\n",
                control_samples / (double)AUDIO_SAMPLE_RATE, useful_rate);
        return -1;
    }

    for (profile = MODEM_PROFILE_SAFE; profile < MODEM_PROFILE_COUNT;
         profile = (enum modem_profile)(profile + 1)) {
        if (test_channel(MODEM_MIN_HZ, MODEM_MAX_HZ, profile, 1.0, 0) != 0)
            return -1;
    }
    if (test_channel(MODEM_MIN_HZ, MODEM_MAX_HZ, MODEM_PROFILE_SAFE,
                     0.9998, 0) != 0 ||
        test_channel(MODEM_MIN_HZ, MODEM_MAX_HZ, MODEM_PROFILE_SAFE,
                     1.0002, 0) != 0)
        return -1;
    if (test_channel(MODEM_MIN_HZ, MODEM_BOOTSTRAP_MAX_HZ,
                     MODEM_PROFILE_SAFE, 1.0, 1) != 0 ||
        test_channel(MODEM_MIN_HZ, MODEM_BOOTSTRAP_MAX_HZ,
                     MODEM_PROFILE_FAST, 0.9998, 1) != 0 ||
        test_channel(MODEM_MIN_HZ, MODEM_BOOTSTRAP_MAX_HZ,
                     MODEM_PROFILE_FAST, 1.0002, 1) != 0)
        return -1;
    return 0;
}
