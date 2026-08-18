#include "modem.h"

#include "audio.h"

#include <complex.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define HEADER_N 16u
#define TRELLIS_GUARD_N 8u
#define RAW_N (HEADER_N + MODEM_PAYLOAD_MAX + 4u + TRELLIS_GUARD_N)
#define CONTROL_RAW_N (HEADER_N + 4u + TRELLIS_GUARD_N)
#define RAW_BITS (RAW_N * 8u)
#define CODE_STEPS (RAW_BITS + 6u)
#define CODE_BITS (CODE_STEPS * 3u)
#define PREAMBLE_SYMBOLS 4u
#define LEAD_N 480u
#define TAIL_N 480u
#define DETECT_LOOKAHEAD 256u
#define CORRELATION_POINTS 160u
#define MIX_FILTER_TAPS 65u
#define OUTPUT_FILTER_TAPS 97u
#define RX_CAPACITY (AUDIO_SAMPLE_RATE * 40u)
#define CSS_MAX_SF 10u
#define CSS_MIN_SF 7u
#define CSS_GUARD_BITS 2u
#define CSS_MAX_M (1u << CSS_MAX_SF)
#define SYNC_CONTROL 64u
#define SYNC_DATA_BASE 192u
#define SYNC_DATA_STEP 192u
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
    int pending;
    size_t pending_start;
    double pending_span;
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
    static const size_t limits[MODEM_PROFILE_COUNT] = {16u, 32u, 64u, 96u};

    if (profile < MODEM_PROFILE_SAFE || profile >= MODEM_PROFILE_COUNT)
        return 0;
    return limits[profile];
}

unsigned modem_profile_spreading_factor(enum modem_profile profile)
{
    static const unsigned factors[MODEM_PROFILE_COUNT] = {10u, 9u, 8u, 7u};

    if (profile < MODEM_PROFILE_SAFE || profile >= MODEM_PROFILE_COUNT)
        return 0;
    return factors[profile];
}

static size_t data_raw_length(enum modem_profile profile)
{
    return HEADER_N + modem_profile_payload_limit(profile) + 4u +
           TRELLIS_GUARD_N;
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

static unsigned symbols_for_raw(size_t raw_length, unsigned sf)
{
    size_t code_bits = (raw_length * 8u + 6u) * 3u;
    unsigned data_bits = sf - CSS_GUARD_BITS;
    return (unsigned)((code_bits + data_bits - 1u) / data_bits);
}

static size_t frame_samples(const band_t *band, size_t raw_length,
                            enum modem_profile profile)
{
    unsigned sf = raw_length == CONTROL_RAW_N
                      ? CSS_MAX_SF
                      : modem_profile_spreading_factor(profile);
    size_t preamble_span = symbol_samples(band, CSS_MAX_SF);
    size_t body_span = symbol_samples(band, sf);
    return LEAD_N + (PREAMBLE_SYMBOLS + 1u) * preamble_span +
           ((size_t)symbols_for_raw(raw_length, sf) + 1u) * body_span +
           TAIL_N;
}

static unsigned body_pilot_value(unsigned sf)
{
    return 1u << (sf - 2u);
}

static double css_phase(double chips, unsigned m)
{
    double wrapped = fmod(chips, m);
    if (wrapped < 0.0)
        wrapped += m;
    return PI * (wrapped * wrapped / m - wrapped);
}

static double symbol_window(size_t index, size_t count,
                            unsigned samples_per_chip)
{
    size_t edge = samples_per_chip * 2u;
    double x;

    if (edge < 48u)
        edge = 48u;
    if (edge > count / 16u)
        edge = count / 16u;
    if (edge == 0)
        return 1.0;
    if (index < edge) {
        x = (double)index / edge;
        return sin(PI * x / 2.0) * sin(PI * x / 2.0);
    }
    if (count - 1u - index < edge) {
        x = (double)(count - 1u - index) / edge;
        return sin(PI * x / 2.0) * sin(PI * x / 2.0);
    }
    return 1.0;
}

static void emit_chirp(const band_t *band, unsigned sf, unsigned value,
                       float *output)
{
    unsigned m = 1u << sf;
    size_t count = symbol_samples(band, sf);
    size_t i;

    for (i = 0; i < count; ++i) {
        double chips = (double)i / band->samples_per_chip + value;
        double carrier = 2.0 * PI * band->center_hz * i /
                         AUDIO_SAMPLE_RATE;
        output[i] = (float)(0.82 * symbol_window(
                                      i, count, band->samples_per_chip) *
                            cos(carrier + css_phase(chips, m)));
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

static int parse_frame(const uint8_t raw[RAW_N], size_t raw_length,
                       modem_frame_t *frame)
{
    uint16_t payload_length;

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
    unsigned state = 0;
    size_t step;

    for (step = 0; step < steps; ++step) {
        unsigned input = step < raw_bits
                             ? (raw[step / 8u] >> (7u - step % 8u)) & 1u
                             : 0u;
        unsigned reg = ((state << 1) | input) & 0x7fu;
        code[step * 3u] = (uint8_t)parity(reg & 0x79u);
        code[step * 3u + 1u] = (uint8_t)parity(reg & 0x5bu);
        code[step * 3u + 2u] = (uint8_t)parity(reg & 0x75u);
        state = reg & 0x3fu;
    }
    return steps * 3u;
}

static int convolutional_decode(const float llr[CODE_BITS], size_t raw_length,
                                uint8_t raw[RAW_N])
{
    float metric[64], next_metric[64];
    uint8_t *trace;
    size_t raw_bits = raw_length * 8u;
    size_t steps = raw_bits + 6u;
    size_t step;
    unsigned state, input;

    trace = malloc(steps * 64u);
    if (!trace)
        return -1;
    for (state = 0; state < 64u; ++state)
        metric[state] = state == 0 ? 0.0f : 1.0e30f;
    for (step = 0; step < steps; ++step) {
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
                unsigned c = parity(reg & 0x75u);
                float branch = (a ? -llr[step * 3u] : llr[step * 3u]) +
                               (b ? -llr[step * 3u + 1u]
                                  : llr[step * 3u + 1u]) +
                               (c ? -llr[step * 3u + 2u]
                                  : llr[step * 3u + 2u]);
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

static void emit_coded_body(const band_t *band, const uint8_t *code,
                            size_t code_bits, unsigned sf, float *output)
{
    unsigned data_bits = sf - CSS_GUARD_BITS;
    unsigned symbol_count =
        (unsigned)((code_bits + data_bits - 1u) / data_bits);
    size_t span = symbol_samples(band, sf);
    unsigned symbol, bit;

    for (symbol = 0; symbol < symbol_count; ++symbol) {
        uint32_t word = 0;
        for (bit = 0; bit < data_bits; ++bit) {
            size_t index = (size_t)bit * symbol_count + symbol;
            word = (word << 1) | (index < code_bits ? code[index] : 0u);
        }
        emit_chirp(band, sf, gray_encode(word) << CSS_GUARD_BITS,
                   output + symbol * span);
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
    size_t raw_length, code_bits, total, position, preamble_span;
    unsigned sf, preamble;
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
    sf = control ? CSS_MAX_SF : modem_profile_spreading_factor(profile);
    serialize_frame(frame, raw);
    code_bits = convolutional_encode(raw, raw_length, code);
    total = frame_samples(&band, raw_length, profile);
    output = calloc(total, sizeof(*output));
    if (!output)
        return -1;
    preamble_span = symbol_samples(&band, CSS_MAX_SF);
    position = LEAD_N;
    for (preamble = 0; preamble < PREAMBLE_SYMBOLS; ++preamble) {
        emit_chirp(&band, CSS_MAX_SF, 0u, output + position);
        position += preamble_span;
    }
    emit_chirp(&band, CSS_MAX_SF, sync_value(profile, control),
               output + position);
    position += preamble_span;
    emit_chirp(&band, sf, body_pilot_value(sf), output + position);
    position += symbol_samples(&band, sf);
    emit_coded_body(&band, code, code_bits, sf, output + position);

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
    size_t data_count, control_count;

    if (profile < MODEM_PROFILE_SAFE || profile >= MODEM_PROFILE_COUNT ||
        make_band(low_hz, high_hz, &band) != 0)
        return 0;
    data_count = frame_samples(&band, data_raw_length(profile), profile);
    control_count = frame_samples(&band, CONTROL_RAW_N, profile);
    return data_count > control_count ? data_count : control_count;
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

static double chirp_correlation(const modem_decoder_t *decoder, double start,
                                double span)
{
    unsigned point;
    unsigned m = 1u << CSS_MAX_SF;
    size_t nominal_span = symbol_samples(&decoder->band, CSS_MAX_SF);
    double in_phase = 0.0, quadrature = 0.0, energy = 0.0;

    if (start < 0.0 || start + span + 1.0 >= decoder->length)
        return 0.0;
    for (point = 0; point < CORRELATION_POINTS; ++point) {
        double fraction = (point + 0.5) / CORRELATION_POINTS;
        double position = start + fraction * span;
        double nominal = fraction * nominal_span;
        double chips = nominal / decoder->band.samples_per_chip;
        double phase = 2.0 * PI * decoder->band.center_hz * nominal /
                           AUDIO_SAMPLE_RATE +
                       css_phase(chips, m);
        double sample = interpolate(decoder->rx, decoder->length, position);
        in_phase += sample * cos(phase);
        quadrature += sample * sin(phase);
        energy += sample * sample;
    }
    if (energy < 1.0e-12)
        return 0.0;
    return sqrt(in_phase * in_phase + quadrature * quadrature) /
           sqrt(energy * CORRELATION_POINTS / 2.0);
}

static double best_chirp_near(const modem_decoder_t *decoder, double target,
                              unsigned radius, double span, double *score)
{
    long offset;
    double best_position = target;
    double best_score = 0.0;

    for (offset = -(long)radius; offset <= (long)radius; ++offset) {
        double position = target + offset;
        double candidate = chirp_correlation(decoder, position, span);
        if (candidate > best_score) {
            best_score = candidate;
            best_position = position;
        }
    }
    if (score)
        *score = best_score;
    return best_position;
}

static double quiet_boundary_near(const modem_decoder_t *decoder,
                                  double target, unsigned radius)
{
    unsigned half_window = decoder->band.samples_per_chip * 4u;
    long offset;
    double best_position = target;
    double best_energy = 1.0e300;

    if (half_window < 16u)
        half_window = 16u;
    for (offset = -(long)radius; offset <= (long)radius; ++offset) {
        long center = lround(target) + offset;
        double energy = 0.0;
        long sample;

        if (center < (long)half_window ||
            center + (long)half_window >= (long)decoder->length)
            continue;
        for (sample = center - (long)half_window;
             sample <= center + (long)half_window; ++sample) {
            double value = decoder->rx[sample];
            energy += value * value;
        }
        if (energy < best_energy) {
            best_energy = energy;
            best_position = center;
        }
    }
    return best_position;
}

static int acquire_preamble(const modem_decoder_t *decoder, size_t coarse,
                            double *start, double *span, double *score)
{
    double first_score, second_score, third_score;
    size_t nominal = symbol_samples(&decoder->band, CSS_MAX_SF);
    unsigned fine = decoder->band.samples_per_chip + 2u;
    unsigned spacing_radius = (unsigned)(nominal / 100u + 8u);
    double first = best_chirp_near(decoder, coarse, fine, nominal,
                                   &first_score);
    double second = best_chirp_near(decoder, first + nominal, spacing_radius,
                                    nominal, &second_score);
    double observed = second - first;
    double third = best_chirp_near(decoder, second + observed,
                                   spacing_radius, observed, &third_score);

    if (first_score < 0.30 || second_score < 0.30 || third_score < 0.30)
        return -1;
    *start = first;
    *span = (third - first) / 2.0;
    *score = (first_score + second_score + third_score) / 3.0;
    if (*span < nominal * 0.98 || *span > nominal * 1.02)
        return -1;
    return 0;
}

static int css_spectrum(const modem_decoder_t *decoder, double start,
                        double span, unsigned sf, double power[CSS_MAX_M],
                        unsigned *peak, double *quality)
{
    unsigned m = 1u << sf;
    size_t nominal_span = symbol_samples(&decoder->band, sf);
    double complex bins[CSS_MAX_M];
    double total_power = 0.0, peak_power = 0.0;
    unsigned chip, k;
    int half = (int)MIX_FILTER_TAPS / 2;

    if (start < 0.0 || start + span + half + 2.0 >= decoder->length)
        return -1;
    for (chip = 0; chip < m; ++chip) {
        double center = start + (chip + 0.5) * span / m;
        double complex analytic = 0.0;
        unsigned tap;
        for (tap = 0; tap < MIX_FILTER_TAPS; ++tap) {
            int displacement = (int)tap - half;
            double position = center + displacement;
            double nominal = (position - start) * nominal_span / span;
            double carrier = 2.0 * PI * decoder->band.center_hz * nominal /
                             AUDIO_SAMPLE_RATE;
            double sample = interpolate(decoder->rx, decoder->length,
                                        position);
            analytic += decoder->band.mix_filter[tap] * sample * 2.0 *
                        (cos(carrier) - I * sin(carrier));
        }
        {
            double chips = chip + 0.5;
            double phase = css_phase(chips, m);
            bins[chip] = analytic * (cos(phase) - I * sin(phase));
        }
    }
    fft(bins, m, 0);
    *peak = 0;
    for (k = 0; k < m; ++k) {
        power[k] = creal(bins[k]) * creal(bins[k]) +
                   cimag(bins[k]) * cimag(bins[k]);
        total_power += power[k];
        if (power[k] > peak_power) {
            peak_power = power[k];
            *peak = k;
        }
    }
    *quality = peak_power /
               (total_power / m > 1.0e-12 ? total_power / m : 1.0e-12);
    return 0;
}

static long signed_bin(unsigned bin, unsigned m)
{
    return bin > m / 2u ? (long)bin - m : (long)bin;
}

static unsigned circular_distance(unsigned a, unsigned b, unsigned m)
{
    unsigned direct = a > b ? a - b : b - a;
    return direct < m - direct ? direct : m - direct;
}

static int decode_sync(const modem_decoder_t *decoder, double start,
                       double span, enum modem_profile *profile,
                       size_t *raw_length)
{
    double power[CSS_MAX_M], quality;
    unsigned preamble_peak, sync_peak, adjusted;
    enum modem_profile candidate;
    long frequency_offset;

    if (css_spectrum(decoder, start, span, CSS_MAX_SF, power,
                     &preamble_peak, &quality) != 0 || quality < 4.0)
        return -1;
    frequency_offset = signed_bin(preamble_peak, CSS_MAX_M);
    if (css_spectrum(decoder, start + PREAMBLE_SYMBOLS * span, span,
                     CSS_MAX_SF, power, &sync_peak, &quality) != 0 ||
        quality < 4.0)
        return -1;
    adjusted = (unsigned)((long)sync_peak - frequency_offset) &
               (CSS_MAX_M - 1u);
    if (circular_distance(adjusted, SYNC_CONTROL, CSS_MAX_M) <= 12u) {
        *profile = MODEM_PROFILE_SAFE;
        *raw_length = CONTROL_RAW_N;
        return 0;
    }
    for (candidate = MODEM_PROFILE_SAFE; candidate < MODEM_PROFILE_COUNT;
         candidate = (enum modem_profile)(candidate + 1)) {
        unsigned expected = sync_value(candidate, 0);
        if (circular_distance(adjusted, expected, CSS_MAX_M) <= 12u) {
            if (candidate != decoder->profile)
                return -1;
            *profile = candidate;
            *raw_length = data_raw_length(candidate);
            return 0;
        }
    }
    return -1;
}

static void spectrum_llrs(const double power[CSS_MAX_M], unsigned sf,
                          float out[CSS_MAX_SF])
{
    double best_zero[CSS_MAX_SF] = {0};
    double best_one[CSS_MAX_SF] = {0};
    unsigned m = 1u << sf;
    unsigned data_bits = sf - CSS_GUARD_BITS;
    unsigned word, bit;

    for (word = 0; word < (1u << data_bits); ++word) {
        unsigned candidate =
            (gray_encode(word) << CSS_GUARD_BITS) & (m - 1u);
        double candidate_power;
        double magnitude;
        int neighbor;

        candidate_power = power[candidate];
        for (neighbor = -1; neighbor <= 1; neighbor += 2) {
            unsigned observed =
                (unsigned)((long)candidate + neighbor) & (m - 1u);
            if (power[observed] > candidate_power)
                candidate_power = power[observed];
        }
        magnitude = sqrt(candidate_power);
        for (bit = 0; bit < data_bits; ++bit) {
            unsigned mask = 1u << (data_bits - 1u - bit);
            double *best = (word & mask) ? &best_one[bit] : &best_zero[bit];
            if (magnitude > *best)
                *best = magnitude;
        }
    }
    for (bit = 0; bit < data_bits; ++bit) {
        double scale = best_one[bit] + best_zero[bit] + 1.0e-12;
        out[bit] = (float)(6.0 * (best_one[bit] - best_zero[bit]) / scale);
    }
}

static int decode_pending(modem_decoder_t *decoder, modem_frame_t *frame)
{
    uint8_t raw[RAW_N];
    float llr[CODE_BITS];
    size_t code_bits = (decoder->pending_raw_length * 8u + 6u) * 3u;
    unsigned sf = decoder->pending_raw_length == CONTROL_RAW_N
                      ? CSS_MAX_SF
                      : modem_profile_spreading_factor(
                            decoder->pending_profile);
    unsigned data_bits = sf - CSS_GUARD_BITS;
    unsigned symbol_count =
        (unsigned)((code_bits + data_bits - 1u) / data_bits);
    double body_span = decoder->pending_span /
                       (1u << (CSS_MAX_SF - sf));
    double body_start = decoder->pending_start +
                        (PREAMBLE_SYMBOLS + 1u) * decoder->pending_span;
    double boundary;
    unsigned boundary_radius;
    unsigned tracking_radius;
    unsigned symbol, bit;

    memset(llr, 0, sizeof(llr));
    boundary_radius = decoder->band.samples_per_chip * 8u + 24u;
    tracking_radius = decoder->band.samples_per_chip + 4u;
    boundary = quiet_boundary_near(decoder, body_start, boundary_radius);
    {
        double power[CSS_MAX_M], quality;
        double next_boundary = quiet_boundary_near(
            decoder, boundary + body_span, tracking_radius);
        double observed_span = next_boundary - boundary;
        unsigned peak;

        if (boundary + observed_span + MIX_FILTER_TAPS / 2u + 2u >=
            decoder->length)
            return 1;
        if (observed_span < body_span * 0.97 ||
            observed_span > body_span * 1.03 ||
            css_spectrum(decoder, boundary, observed_span, sf, power,
                         &peak, &quality) != 0 ||
            quality < 2.0)
            return -1;
        if (circular_distance(peak, body_pilot_value(sf), 1u << sf) > 2u)
            return -1;
        boundary = next_boundary;
    }
    for (symbol = 0; symbol < symbol_count; ++symbol) {
        double power[CSS_MAX_M], quality = 0.0;
        float bits[CSS_MAX_SF];
        double next_boundary = quiet_boundary_near(
            decoder, boundary + body_span, tracking_radius);
        double observed_span = next_boundary - boundary;
        unsigned peak;
        if (boundary + observed_span + MIX_FILTER_TAPS / 2u + 2u >=
            decoder->length)
            return 1;
        if (observed_span < body_span * 0.97 ||
            observed_span > body_span * 1.03 ||
            css_spectrum(decoder, boundary, observed_span, sf, power,
                         &peak, &quality) != 0 ||
            quality < 1.8) {
            return -1;
        }
        (void)peak;
        spectrum_llrs(power, sf, bits);
        for (bit = 0; bit < data_bits; ++bit) {
            size_t index = (size_t)bit * symbol_count + symbol;
            if (index < code_bits)
                llr[index] = bits[bit];
        }
        boundary = next_boundary;
    }
    if (convolutional_decode(llr, decoder->pending_raw_length, raw) != 0)
        return -1;
    return parse_frame(raw, decoder->pending_raw_length, frame);
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
    decoder->peak_sync = 0.0;
    decoder->candidates = 0;
    decoder->rejected = 0;
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
        if (decoder->pending_start >= count)
            decoder->pending_start -= count;
        else
            decoder->pending = 0;
    }
}

int modem_decoder_feed(modem_decoder_t *decoder, const float *samples,
                       size_t count, modem_frame_t *frame)
{
    size_t nominal;
    size_t prefix;
    unsigned scan_step;

    if (!decoder || !frame || (count && !samples))
        return -1;
    nominal = symbol_samples(&decoder->band, CSS_MAX_SF);
    prefix = (PREAMBLE_SYMBOLS + 1u) * nominal;
    scan_step = decoder->band.samples_per_chip;
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
        size_t best_start = 0, position;
        double best = 0.0;

        for (position = decoder->search_position; position <= max_start;
             position += scan_step) {
            double first = chirp_correlation(decoder, position, nominal);
            double score;
            if (first > decoder->peak_sync)
                decoder->peak_sync = first;
            if (first < 0.20)
                continue;
            score = (first + chirp_correlation(decoder, position + nominal,
                                               nominal)) /
                    2.0;
            if (score > decoder->peak_sync)
                decoder->peak_sync = score;
            if (score > best) {
                best = score;
                best_start = position;
            }
        }
        decoder->search_position = max_start + scan_step;
        if (best < 0.35)
            break;
        {
            double start, span, score;
            enum modem_profile profile;
            size_t raw_length;
            unsigned sf, body_symbols;
            double body_span, body_end;

            ++decoder->candidates;
            if (acquire_preamble(decoder, best_start, &start, &span,
                                 &score) != 0 ||
                decode_sync(decoder, start, span, &profile,
                            &raw_length) != 0) {
                ++decoder->rejected;
                discard_samples(decoder, best_start + scan_step);
                continue;
            }
            if (score > decoder->peak_sync)
                decoder->peak_sync = score;
            sf = raw_length == CONTROL_RAW_N
                     ? CSS_MAX_SF
                     : modem_profile_spreading_factor(profile);
            body_symbols = symbols_for_raw(raw_length, sf);
            body_span = span / (1u << (CSS_MAX_SF - sf));
            body_end = start + (PREAMBLE_SYMBOLS + 1u) * span +
                       (body_symbols + 1u) * body_span;
            decoder->pending = 1;
            decoder->pending_start = (size_t)start;
            decoder->pending_span = span;
            decoder->pending_profile = profile;
            decoder->pending_raw_length = raw_length;
            decoder->pending_needed =
                (size_t)ceil(body_end) + DETECT_LOOKAHEAD;
            if (decoder->length < decoder->pending_needed)
                return 0;
            return modem_decoder_feed(decoder, NULL, 0, frame);
        }
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
    channel_count = 137u + resampled_count + 31u + 1024u;
    channel = calloc(channel_count, sizeof(*channel));
    decoder = modem_decoder_create(low, high);
    if (!channel || !decoder ||
        modem_decoder_set_profile(decoder, profile) != 0)
        goto done;
    for (i = 0; i < channel_count; ++i) {
        double noise = ((int)(test_random(&random) >> 16) - 32768) / 32768.0;
        channel[i] = (float)(noise * 0.006);
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
        channel[137u + i + 31u] += sample * 0.07f;
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
                "  decoder: got=%d, sync=%.3f, candidates=%u, rejected=%u\n",
                got, activity.peak_sync, activity.candidates,
                activity.rejected);
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
    size_t code_bits, i;

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

    for (profile = MODEM_PROFILE_SAFE; profile < MODEM_PROFILE_COUNT;
         profile = (enum modem_profile)(profile + 1)) {
        if (test_channel(MODEM_MIN_HZ, MODEM_MAX_HZ, profile, 1.0, 0) != 0 ||
            test_channel(3500u, 8500u, profile, 1.0, 0) != 0 ||
            test_channel(MODEM_MIN_HZ, MODEM_MAX_HZ, profile, 0.998, 0) != 0 ||
            test_channel(MODEM_MIN_HZ, MODEM_MAX_HZ, profile, 1.002, 0) != 0)
            return -1;
    }
    if (test_channel(MODEM_MIN_HZ, MODEM_BOOTSTRAP_MAX_HZ,
                     MODEM_PROFILE_SAFE, 1.0, 1) != 0 ||
        test_channel(MODEM_MIN_HZ, MODEM_BOOTSTRAP_MAX_HZ,
                     MODEM_PROFILE_FAST, 0.998, 1) != 0 ||
        test_channel(MODEM_MIN_HZ, MODEM_BOOTSTRAP_MAX_HZ,
                     MODEM_PROFILE_FAST, 1.002, 1) != 0)
        return -1;
    return 0;
}
