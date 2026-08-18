#include "audio.h"
#include "modem.h"
#include "packet.h"
#include "tunnel.h"

#include <errno.h>
#ifdef __linux__
#include <pwd.h>
#endif
#include <math.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define FLAG_MORE 1u
#define IDLE_POLL_MS 1000u
#define LINK_TIMEOUT_MS 120000u
#define TURNAROUND_MS 250u
#define OUTPUT_TAIL_MS 180u
#define HANDSHAKE_SETTLE_MS 2500u
#define RATE_INTERVAL_MS 5000u
#define PROMOTE_CLEAN_TRANSACTIONS 8u

enum client_state { CLIENT_SEARCH, CLIENT_WAIT_READY, CLIENT_READY,
                    CLIENT_WAIT_RESPONSE, CLIENT_WAIT_PROFILE };
enum gateway_state { GATEWAY_LISTEN, GATEWAY_WAIT_CONFIRM,
                     GATEWAY_CONNECTED };

typedef struct {
    int gateway;
    int configure;
    unsigned low_hz;
    unsigned high_hz;
    const char *input_device;
    const char *output_device;
} options_t;

typedef struct {
    options_t options;
    audio_t *audio;
    tunnel_t *tunnel;
    modem_decoder_t *bootstrap_decoder;
    modem_decoder_t *link_decoder;
    int link_decoder_enabled;
    int link_matches_bootstrap;
    enum client_state client_state;
    enum gateway_state gateway_state;
    uint32_t session;
    uint16_t sequence;
    uint16_t last_gateway_sequence;
    unsigned selected_low;
    unsigned selected_high;
    enum modem_profile profile;
    enum modem_profile profile_target;
    int profile_resume_request;
    unsigned clean_transactions;
    uint64_t profile_attempts;
    uint64_t profile_successes;
    uint64_t next_action;
    uint64_t last_receive;
    unsigned retries;
    modem_frame_t pending_request;
    modem_frame_t cached_response;
    int cached_response_valid;
    int response_fragment_pending;
    packet_queue_t outbound;
    packet_sender_t sender;
    packet_receiver_t receiver;
    uint64_t packets_from_tun;
    uint64_t packets_to_tun;
    uint64_t dropped_packets;
    uint64_t retries_sent;
    uint64_t last_stats;
    uint64_t bytes_sent;
    uint64_t bytes_received;
    uint64_t rate_bytes_sent;
    uint64_t rate_bytes_received;
    uint64_t last_rate;
    double input_square_sum;
    float input_peak;
    uint64_t input_samples;
    uint64_t muted_input_samples;
    uint64_t next_audio_status;
    uint64_t ignore_input_until;
} app_t;

static volatile sig_atomic_t stopping;

static uint64_t milliseconds(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (uint64_t)now.tv_sec * 1000u + (uint64_t)now.tv_nsec / 1000000u;
}

static void log_line(const app_t *app, const char *format, ...)
{
    va_list args;
    time_t now = time(NULL);
    struct tm local;
    char stamp[16];

    localtime_r(&now, &local);
    strftime(stamp, sizeof(stamp), "%H:%M:%S", &local);
    fprintf(stderr, "[%s] %-7s ", stamp,
            app->options.gateway ? "gateway" : "client");
    va_start(args, format);
    vfprintf(stderr, format, args);
    va_end(args);
    fputc('\n', stderr);
}

static const char *profile_name(enum modem_profile profile)
{
    static const char *const names[MODEM_PROFILE_COUNT] = {
        "safe", "robust", "balanced", "fast"};

    if (profile < MODEM_PROFILE_SAFE || profile >= MODEM_PROFILE_COUNT)
        return "invalid";
    return names[profile];
}

static size_t fragment_data_bytes(const modem_frame_t *frame)
{
    return frame->payload_len >= 8u ? frame->payload_len - 8u : 0u;
}

static void handle_signal(int signal_number)
{
    (void)signal_number;
    stopping = 1;
}

static uint32_t new_session(void)
{
    uint32_t value = 0;
    FILE *random = fopen("/dev/urandom", "rb");
    if (random) {
        if (fread(&value, sizeof(value), 1, random) != 1)
            value = 0;
        fclose(random);
    }
    if (value == 0)
        value = (uint32_t)time(NULL) ^ (uint32_t)getpid() ^
                (uint32_t)milliseconds();
    return value ? value : 1u;
}

static int parse_band(const char *text, unsigned *low, unsigned *high)
{
    char trailing;
    if ((sscanf(text, "%u:%u%c", low, high, &trailing) == 2 ||
         sscanf(text, "%u-%u%c", low, high, &trailing) == 2) &&
        *low >= MODEM_MIN_HZ && *high <= MODEM_MAX_HZ &&
        *low + MODEM_MIN_BANDWIDTH_HZ <= *high)
        return 0;
    return -1;
}

static void usage(FILE *stream, const char *program)
{
    fprintf(stream,
        "Usage: %s [--client | --gateway] [options]\n"
        "\n"
        "Modes:\n"
        "  --client              route this machine through an audio gateway (default)\n"
        "  --gateway             share this machine's internet connection\n"
        "\n"
        "Options:\n"
        "  --band LOW:HIGH       requested audio band in Hz (default 2000:12000)\n"
        "  --input DEVICE        ALSA name or CoreAudio device UID\n"
        "  --output DEVICE       ALSA name or CoreAudio device UID\n"
        "  --no-config           create TUN/utun but do not alter routes or NAT\n"
        "  --self-test           test the codec without audio devices or root\n"
        "  --help                show this help\n",
        program);
}

static int parse_options(int argc, char **argv, options_t *options,
                         int *self_test)
{
    int i;
    memset(options, 0, sizeof(*options));
    options->configure = 1;
    options->low_hz = MODEM_MIN_HZ;
    options->high_hz = MODEM_DEFAULT_MAX_HZ;
    *self_test = 0;
    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--client") == 0)
            options->gateway = 0;
        else if (strcmp(argv[i], "--gateway") == 0)
            options->gateway = 1;
        else if (strcmp(argv[i], "--no-config") == 0)
            options->configure = 0;
        else if (strcmp(argv[i], "--self-test") == 0)
            *self_test = 1;
        else if (strcmp(argv[i], "--band") == 0 && i + 1 < argc) {
            if (parse_band(argv[++i], &options->low_hz,
                           &options->high_hz) != 0) {
                fprintf(stderr,
                        "invalid band; use LOW:HIGH within 2000:12000 and at least 4000 Hz wide\n");
                return -1;
            }
        } else if (strcmp(argv[i], "--input") == 0 && i + 1 < argc) {
            options->input_device = argv[++i];
        } else if (strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
            options->output_device = argv[++i];
        } else if (strcmp(argv[i], "--help") == 0) {
            usage(stdout, argv[0]);
            exit(0);
        } else {
            fprintf(stderr, "unknown or incomplete option: %s\n", argv[i]);
            usage(stderr, argv[0]);
            return -1;
        }
    }
    return 0;
}

static int send_frame(app_t *app, const modem_frame_t *frame, int bootstrap)
{
    float *samples = NULL;
    size_t count = 0;
    char error[256];
    struct timespec turnaround = {
        TURNAROUND_MS / 1000u,
        (long)(TURNAROUND_MS % 1000u) * 1000000L};
    unsigned low = bootstrap ? MODEM_MIN_HZ : app->selected_low;
    unsigned high = bootstrap ? MODEM_BOOTSTRAP_MAX_HZ : app->selected_high;

    if (modem_encode(frame, low, high, app->profile, &samples, &count) != 0) {
        log_line(app, "cannot encode audio frame");
        return -1;
    }
    while (nanosleep(&turnaround, &turnaround) != 0 && errno == EINTR &&
           !stopping)
        ;
    if (stopping) {
        modem_free_samples(samples);
        return -1;
    }
    if (audio_send(app->audio, samples, count, error, sizeof(error)) != 0) {
        log_line(app, "%s", error);
        modem_free_samples(samples);
        return -1;
    }
    app->ignore_input_until =
        milliseconds() +
        ((uint64_t)count * 1000u + AUDIO_SAMPLE_RATE - 1u) /
            AUDIO_SAMPLE_RATE +
        OUTPUT_TAIL_MS;
    modem_free_samples(samples);
    return 0;
}

static uint64_t retry_delay(const app_t *app)
{
    size_t burst = modem_burst_samples(app->selected_low,
                                       app->selected_high, app->profile);
    /* A complete request and response, plus device and room latency. */
    return 3000u + (uint64_t)burst * 2000u / AUDIO_SAMPLE_RATE;
}

static uint64_t bootstrap_retry_delay(void)
{
    size_t burst = modem_burst_samples(MODEM_MIN_HZ,
                                       MODEM_BOOTSTRAP_MAX_HZ,
                                       MODEM_PROFILE_SAFE);
    return 3000u + (uint64_t)burst * 2000u / AUDIO_SAMPLE_RATE;
}

static int set_data_profile(app_t *app, enum modem_profile profile)
{
    modem_decoder_t *decoder;

    if (profile < MODEM_PROFILE_SAFE || profile >= MODEM_PROFILE_COUNT)
        return -1;
    decoder = app->link_matches_bootstrap ? app->bootstrap_decoder
                                          : app->link_decoder;
    if (modem_decoder_set_profile(decoder, profile) != 0)
        return -1;
    app->profile = profile;
    app->clean_transactions = 0;
    app->profile_attempts = 0;
    app->profile_successes = 0;
    return 0;
}

static int set_link_band(app_t *app, unsigned low, unsigned high)
{
    if (modem_decoder_set_band(app->link_decoder, low, high) != 0)
        return -1;
    app->selected_low = low;
    app->selected_high = high;
    app->link_decoder_enabled = 1;
    app->link_matches_bootstrap =
        low == MODEM_MIN_HZ && high == MODEM_BOOTSTRAP_MAX_HZ;
    return set_data_profile(app, MODEM_PROFILE_SAFE);
}

static void reset_profile(app_t *app)
{
    app->profile = MODEM_PROFILE_SAFE;
    app->clean_transactions = 0;
    app->profile_attempts = 0;
    app->profile_successes = 0;
    (void)modem_decoder_set_profile(app->bootstrap_decoder,
                                    MODEM_PROFILE_SAFE);
    (void)modem_decoder_set_profile(app->link_decoder, MODEM_PROFILE_SAFE);
}

static void reset_client(app_t *app, const char *reason)
{
    if (reason)
        log_line(app, "link down: %s; searching again", reason);
    app->client_state = CLIENT_SEARCH;
    app->session = new_session();
    app->link_decoder_enabled = 0;
    app->sequence = 1;
    app->next_action = 0;
    app->retries = 0;
    reset_profile(app);
    packet_receiver_init(&app->receiver);
}

static void reset_gateway(app_t *app, const char *reason)
{
    if (reason)
        log_line(app, "link down: %s; listening again", reason);
    app->gateway_state = GATEWAY_LISTEN;
    app->session = 0;
    app->link_decoder_enabled = 0;
    app->sequence = 1;
    app->last_gateway_sequence = 0;
    app->cached_response_valid = 0;
    app->response_fragment_pending = 0;
    reset_profile(app);
    packet_receiver_init(&app->receiver);
}

static void send_hello(app_t *app)
{
    modem_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.type = MODEM_HELLO;
    frame.session = app->session;
    frame.band_low = (uint16_t)app->options.low_hz;
    frame.band_high = (uint16_t)app->options.high_hz;
    if (send_frame(app, &frame, 1) == 0) {
        log_line(app, "connection request sent (session %08x, band %u-%u Hz)",
                 app->session, app->options.low_hz, app->options.high_hz);
        app->next_action = milliseconds() + bootstrap_retry_delay();
    }
}

static void send_offer(app_t *app)
{
    modem_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.type = MODEM_OFFER;
    frame.session = app->session;
    frame.band_low = (uint16_t)app->selected_low;
    frame.band_high = (uint16_t)app->selected_high;
    if (send_frame(app, &frame, 1) == 0)
        app->next_action = milliseconds() + bootstrap_retry_delay();
}

static void send_confirm(app_t *app)
{
    modem_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.type = MODEM_CONFIRM;
    frame.session = app->session;
    frame.band_low = (uint16_t)app->selected_low;
    frame.band_high = (uint16_t)app->selected_high;
    if (send_frame(app, &frame, 1) == 0)
        app->next_action = milliseconds() + retry_delay(app);
}

static void send_ready(app_t *app)
{
    modem_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.type = MODEM_READY;
    frame.session = app->session;
    frame.band_low = (uint16_t)app->selected_low;
    frame.band_high = (uint16_t)app->selected_high;
    (void)send_frame(app, &frame, 1);
}

static void send_profile_control(app_t *app, enum modem_profile profile,
                                 int acknowledgement, uint16_t sequence,
                                 int request_cached)
{
    modem_frame_t frame;

    memset(&frame, 0, sizeof(frame));
    frame.type = MODEM_PROFILE_SELECT;
    frame.flags = (uint8_t)profile;
    frame.session = app->session;
    frame.seq = sequence;
    if (acknowledgement) {
        frame.ack = frame.seq;
        frame.band_low = request_cached ? 1u : 0u;
    }
    if (send_frame(app, &frame, 1) == 0 && !acknowledgement)
        app->next_action = milliseconds() + bootstrap_retry_delay();
}

static void begin_profile_change(app_t *app, enum modem_profile target,
                                 int resume_request, const char *reason)
{
    log_line(app, "requesting %s profile (SF%u, %zu-byte frames): %s",
             profile_name(target), modem_profile_spreading_factor(target),
             modem_profile_payload_limit(target), reason);
    app->profile_target = target;
    app->profile_resume_request = resume_request;
    app->client_state = CLIENT_WAIT_PROFILE;
    app->retries = 0;
    send_profile_control(app, target, 0, app->sequence, 0);
}

static void deliver_fragment(app_t *app, const modem_frame_t *frame)
{
    packet_t packet;
    int result;
    if (frame->payload_len == 0)
        return;
    result = packet_receiver_add(&app->receiver, frame->payload,
                                 frame->payload_len, &packet);
    if (result == 1) {
        ssize_t wrote = tunnel_write(app->tunnel, packet.data, packet.len);
        if (wrote == (ssize_t)packet.len)
            ++app->packets_to_tun;
        else {
            ++app->dropped_packets;
            log_line(app, "dropped received IP packet: %s", strerror(errno));
        }
    } else if (result < 0) {
        ++app->dropped_packets;
        log_line(app, "discarded invalid packet fragment");
    }
}

static void stage_client_request(app_t *app)
{
    memset(&app->pending_request, 0, sizeof(app->pending_request));
    app->pending_request.type = MODEM_REQUEST;
    app->pending_request.session = app->session;
    app->pending_request.seq = app->sequence;
    app->pending_request.payload_len = (uint16_t)packet_sender_fragment(
        &app->sender, &app->outbound, app->pending_request.payload,
        modem_profile_payload_limit(app->profile));
    if (app->sender.active || app->outbound.count)
        app->pending_request.flags |= FLAG_MORE;
}

static void stage_gateway_response(app_t *app, uint16_t sequence)
{
    memset(&app->cached_response, 0, sizeof(app->cached_response));
    app->cached_response.type = MODEM_RESPONSE;
    app->cached_response.session = app->session;
    app->cached_response.seq = sequence;
    app->cached_response.ack = sequence;
    app->cached_response.payload_len = (uint16_t)packet_sender_fragment(
        &app->sender, &app->outbound, app->cached_response.payload,
        modem_profile_payload_limit(app->profile));
    app->response_fragment_pending = app->cached_response.payload_len != 0;
    if (app->response_fragment_pending || app->outbound.count)
        app->cached_response.flags |= FLAG_MORE;
}

static void client_frame(app_t *app, const modem_frame_t *frame)
{
    uint64_t now = milliseconds();

    if (frame->session != app->session)
        return;
    if (app->client_state == CLIENT_SEARCH && frame->type == MODEM_OFFER) {
        if (frame->band_low < app->options.low_hz ||
            frame->band_high > app->options.high_hz ||
            frame->band_low + MODEM_MIN_BANDWIDTH_HZ > frame->band_high ||
            set_link_band(app, frame->band_low, frame->band_high) != 0) {
            log_line(app, "ignored offer with incompatible band");
            return;
        }
        log_line(app, "gateway offer received; negotiated %u-%u Hz",
                 app->selected_low, app->selected_high);
        send_confirm(app);
        app->client_state = CLIENT_WAIT_READY;
        app->retries = 0;
        return;
    }
    if (app->client_state == CLIENT_WAIT_READY && frame->type == MODEM_READY) {
        app->client_state = CLIENT_READY;
        app->sequence = 1;
        app->next_action = now + HANDSHAKE_SETTLE_MS;
        app->last_receive = now;
        app->last_rate = now;
        app->rate_bytes_sent = app->bytes_sent;
        app->rate_bytes_received = app->bytes_received;
        log_line(app,
                 "link up (session %08x, %s profile: SF%u/%zu bytes, %u-%u Hz); settling %u ms",
                 app->session, profile_name(app->profile),
                 modem_profile_spreading_factor(app->profile),
                 modem_profile_payload_limit(app->profile), app->selected_low,
                 app->selected_high, HANDSHAKE_SETTLE_MS);
        return;
    }
    if (app->client_state == CLIENT_WAIT_PROFILE &&
        frame->type == MODEM_PROFILE_SELECT &&
        frame->flags == (uint8_t)app->profile_target &&
        frame->seq == app->sequence && frame->ack == frame->seq &&
        frame->band_low <= 1u) {
        int resume_request = app->profile_resume_request;
        int request_cached = frame->band_low == 1u;

        if (set_data_profile(app, app->profile_target) != 0) {
            reset_client(app, "invalid profile acknowledgement");
            return;
        }
        log_line(app, "profile active: %s (SF%u, %zu-byte frames)",
                 profile_name(app->profile),
                 modem_profile_spreading_factor(app->profile),
                 modem_profile_payload_limit(app->profile));
        app->profile_resume_request = 0;
        app->retries = 0;
        if (resume_request) {
            modem_frame_t retry_frame;
            const modem_frame_t *outgoing;

            if (request_cached) {
                memset(&retry_frame, 0, sizeof(retry_frame));
                retry_frame.type = MODEM_REQUEST;
                retry_frame.session = app->session;
                retry_frame.seq = app->sequence;
                outgoing = &retry_frame;
            } else {
                stage_client_request(app);
                outgoing = &app->pending_request;
            }
            if (send_frame(app, outgoing, 0) == 0) {
                app->client_state = CLIENT_WAIT_RESPONSE;
                app->next_action = milliseconds() + retry_delay(app);
                if (request_cached)
                    log_line(app,
                             "requesting cached response for link sequence %u using %s profile",
                             app->sequence, profile_name(app->profile));
                else
                    log_line(app,
                             "link sequence %u re-sent using %s profile (%zu bytes up)",
                             app->sequence, profile_name(app->profile),
                             fragment_data_bytes(&app->pending_request));
            }
        } else {
            app->client_state = CLIENT_READY;
            app->next_action = now;
        }
        return;
    }
    if (app->client_state == CLIENT_WAIT_RESPONSE &&
        frame->type == MODEM_RESPONSE && frame->ack == app->sequence) {
        size_t sent_bytes = fragment_data_bytes(&app->pending_request);
        size_t received_bytes = fragment_data_bytes(frame);
        int carried_data = sent_bytes != 0 || received_bytes != 0;

        app->bytes_sent += sent_bytes;
        app->bytes_received += received_bytes;
        packet_sender_commit(&app->sender);
        deliver_fragment(app, frame);
        if (carried_data) {
            ++app->profile_attempts;
            ++app->profile_successes;
            if (app->retries == 0)
                ++app->clean_transactions;
            else
                app->clean_transactions = 0;
            log_line(app,
                     "link sequence %u acknowledged (%zu bytes up, %zu bytes down)",
                     app->sequence, sent_bytes, received_bytes);
        }
        if (++app->sequence == 0)
            app->sequence = 1;
        app->client_state = CLIENT_READY;
        app->last_receive = now;
        app->retries = 0;
        app->next_action = (app->sender.active || app->outbound.count ||
                            (frame->flags & FLAG_MORE))
                               ? now
                               : now + IDLE_POLL_MS;
        if (carried_data &&
            app->clean_transactions >= PROMOTE_CLEAN_TRANSACTIONS &&
            app->profile < MODEM_PROFILE_FAST) {
            begin_profile_change(
                app, (enum modem_profile)(app->profile + 1), 0,
                "clean transfer streak");
        }
    }
}

static void gateway_frame(app_t *app, const modem_frame_t *frame)
{
    uint64_t now = milliseconds();

    if (frame->type == MODEM_HELLO) {
        unsigned low = frame->band_low > app->options.low_hz
                           ? frame->band_low : app->options.low_hz;
        unsigned high = frame->band_high < app->options.high_hz
                            ? frame->band_high : app->options.high_hz;
        if (low + MODEM_MIN_BANDWIDTH_HZ > high)
            return;
        if (app->gateway_state == GATEWAY_LISTEN ||
            frame->session != app->session) {
            if (app->gateway_state == GATEWAY_CONNECTED)
                log_line(app, "client requested a fresh link; restarting handshake");
            app->session = frame->session;
            if (set_link_band(app, low, high) != 0)
                return;
            app->gateway_state = GATEWAY_WAIT_CONFIRM;
            app->retries = 0;
            app->sequence = 1;
            app->last_gateway_sequence = 0;
            app->cached_response_valid = 0;
            app->response_fragment_pending = 0;
            packet_receiver_init(&app->receiver);
            log_line(app, "request heard (session %08x); offering %u-%u Hz",
                     app->session, low, high);
        }
        send_offer(app);
        return;
    }
    if (frame->session != app->session)
        return;
    if (app->gateway_state == GATEWAY_WAIT_CONFIRM &&
        frame->type == MODEM_CONFIRM) {
        send_ready(app);
        app->gateway_state = GATEWAY_CONNECTED;
        app->sequence = 1;
        app->last_gateway_sequence = 0;
        app->last_receive = now;
        app->cached_response_valid = 0;
        app->last_rate = now;
        app->rate_bytes_sent = app->bytes_sent;
        app->rate_bytes_received = app->bytes_received;
        packet_receiver_init(&app->receiver);
        log_line(app,
                 "link up (session %08x, %s profile: SF%u/%zu bytes, %u-%u Hz)",
                 app->session, profile_name(app->profile),
                 modem_profile_spreading_factor(app->profile),
                 modem_profile_payload_limit(app->profile), app->selected_low,
                 app->selected_high);
        return;
    }
    if (app->gateway_state == GATEWAY_CONNECTED &&
        frame->type == MODEM_CONFIRM) {
        send_ready(app);
        return;
    }
    if (app->gateway_state == GATEWAY_CONNECTED &&
        frame->type == MODEM_PROFILE_SELECT &&
        frame->flags < MODEM_PROFILE_COUNT && frame->ack == 0) {
        enum modem_profile target = (enum modem_profile)frame->flags;
        enum modem_profile previous = app->profile;
        int request_cached = frame->seq == app->last_gateway_sequence &&
                             app->cached_response_valid;

        send_profile_control(app, target, 1, frame->seq, request_cached);
        if (target != previous && app->response_fragment_pending &&
            (target > previous || !request_cached)) {
            app->bytes_sent += fragment_data_bytes(&app->cached_response);
            packet_sender_commit(&app->sender);
            app->response_fragment_pending = 0;
        }
        if (target != previous && set_data_profile(app, target) == 0) {
            if (target < previous && request_cached &&
                app->response_fragment_pending)
                stage_gateway_response(app, app->last_gateway_sequence);
            log_line(app,
                     "profile active: %s (SF%u, %zu-byte frames)",
                     profile_name(app->profile),
                     modem_profile_spreading_factor(app->profile),
                     modem_profile_payload_limit(app->profile));
        }
        app->last_receive = now;
        return;
    }
    if (app->gateway_state != GATEWAY_CONNECTED ||
        frame->type != MODEM_REQUEST)
        return;
    if (frame->seq == app->last_gateway_sequence &&
        app->cached_response_valid) {
        (void)send_frame(app, &app->cached_response, 0);
        ++app->retries_sent;
        log_line(app, "re-sent response for link sequence %u", frame->seq);
        app->last_receive = now;
        return;
    }
    if (frame->seq != app->sequence)
        return;

    if (app->response_fragment_pending)
        app->bytes_sent += fragment_data_bytes(&app->cached_response);
    if (app->response_fragment_pending)
        packet_sender_commit(&app->sender);
    app->response_fragment_pending = 0;
    app->bytes_received += fragment_data_bytes(frame);
    deliver_fragment(app, frame);
    stage_gateway_response(app, frame->seq);
    if (frame->payload_len || app->cached_response.payload_len)
        log_line(app,
                 "link sequence %u received (%zu bytes up); responding with %zu bytes down",
                 frame->seq, fragment_data_bytes(frame),
                 fragment_data_bytes(&app->cached_response));
    (void)send_frame(app, &app->cached_response, 0);
    app->cached_response_valid = 1;
    app->last_gateway_sequence = frame->seq;
    if (++app->sequence == 0)
        app->sequence = 1;
    app->last_receive = now;
}

static void dispatch_frame(app_t *app, const modem_frame_t *frame)
{
    if (app->options.gateway)
        gateway_frame(app, frame);
    else
        client_frame(app, frame);
}

static void pump_decoder(app_t *app, modem_decoder_t *decoder,
                         const float *samples, size_t count)
{
    modem_frame_t frame;
    int result = modem_decoder_feed(decoder, samples, count, &frame);
    while (result == 1) {
        dispatch_frame(app, &frame);
        result = modem_decoder_feed(decoder, NULL, 0, &frame);
    }
}

static void pump_audio(app_t *app)
{
    float samples[4096];
    size_t count;
    while ((count = audio_read(app->audio, samples,
                               sizeof(samples) / sizeof(samples[0]))) != 0) {
        size_t i;
        if (milliseconds() < app->ignore_input_until) {
            app->muted_input_samples += count;
            continue;
        }
        for (i = 0; i < count; ++i) {
            float absolute = fabsf(samples[i]);
            app->input_square_sum += (double)samples[i] * samples[i];
            if (absolute > app->input_peak)
                app->input_peak = absolute;
        }
        app->input_samples += count;
        pump_decoder(app, app->bootstrap_decoder, samples, count);
        if (app->link_decoder_enabled && !app->link_matches_bootstrap)
            pump_decoder(app, app->link_decoder, samples, count);
    }
}

static void maybe_log_audio_status(app_t *app, uint64_t now)
{
    modem_activity_t activity, link_activity;
    int connecting, waiting_for_peer, report;

    if (now < app->next_audio_status)
        return;
    app->next_audio_status = now + 5000u;
    modem_decoder_take_activity(app->bootstrap_decoder, &activity);
    if (app->link_decoder_enabled && !app->link_matches_bootstrap) {
        modem_decoder_take_activity(app->link_decoder, &link_activity);
        if (link_activity.peak_sync > activity.peak_sync)
            activity.peak_sync = link_activity.peak_sync;
        activity.candidates += link_activity.candidates;
        activity.rejected += link_activity.rejected;
        activity.timing_rejected += link_activity.timing_rejected;
        activity.sync_rejected += link_activity.sync_rejected;
        activity.pilot_rejected += link_activity.pilot_rejected;
        activity.payload_rejected += link_activity.payload_rejected;
    }
    connecting = app->options.gateway
                     ? app->gateway_state != GATEWAY_CONNECTED
                     : app->client_state != CLIENT_READY &&
                           app->client_state != CLIENT_WAIT_RESPONSE &&
                           app->client_state != CLIENT_WAIT_PROFILE;
    waiting_for_peer = app->options.gateway
                           ? app->gateway_state == GATEWAY_CONNECTED
                           : app->client_state == CLIENT_WAIT_RESPONSE ||
                                 app->client_state == CLIENT_WAIT_PROFILE;
    report = connecting || activity.rejected ||
             (waiting_for_peer && activity.peak_sync > 0.0);
    if (report) {
        if (app->input_samples) {
            double rms = sqrt(app->input_square_sum / app->input_samples);
            double rms_db = 20.0 * log10(rms > 1.0e-6 ? rms : 1.0e-6);
            double peak_db = 20.0 * log10(app->input_peak > 1.0e-6f
                                              ? app->input_peak
                                              : 1.0e-6f);
            if (activity.rejected)
                log_line(app,
                         "input level: %.0f dBFS RMS, %.0f dBFS peak; best modem sync %.2f; rejected %u candidate(s): timing %u, sync %u, pilot %u, payload/CRC %u",
                         rms_db, peak_db, activity.peak_sync,
                         activity.rejected, activity.timing_rejected,
                         activity.sync_rejected, activity.pilot_rejected,
                         activity.payload_rejected);
            else
                log_line(app,
                         "input level: %.0f dBFS RMS, %.0f dBFS peak; best modem sync %.2f",
                         rms_db, peak_db, activity.peak_sync);
        } else if (app->muted_input_samples) {
            log_line(app,
                     "local transmit active; microphone decoding temporarily muted");
        } else {
            log_line(app, "audio input produced no samples");
        }
    }
    app->input_square_sum = 0.0;
    app->input_peak = 0.0f;
    app->input_samples = 0;
    app->muted_input_samples = 0;
}

static void pump_tunnel(app_t *app)
{
    uint8_t packet[PACKET_MAX];
    while (app->outbound.count < PACKET_QUEUE_CAP) {
        ssize_t got = tunnel_read(app->tunnel, packet, sizeof(packet));
        if (got < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
                log_line(app, "TUN read failed: %s", strerror(errno));
            break;
        }
        if (got == 0)
            break;
        if (packet_queue_push(&app->outbound, packet, (size_t)got) == 0)
            ++app->packets_from_tun;
        else
            ++app->dropped_packets;
    }
}

static void client_tick(app_t *app, uint64_t now)
{
    if (app->client_state == CLIENT_SEARCH && now >= app->next_action) {
        send_hello(app);
    } else if (app->client_state == CLIENT_WAIT_READY &&
               now >= app->next_action) {
        if (++app->retries > 4u) {
            reset_client(app, "gateway did not confirm the link");
        } else {
            log_line(app, "re-sending connection confirmation (%u/4)",
                     app->retries);
            send_confirm(app);
        }
    } else if (app->client_state == CLIENT_READY && now >= app->next_action) {
        stage_client_request(app);
        if (send_frame(app, &app->pending_request, 0) == 0) {
            app->client_state = CLIENT_WAIT_RESPONSE;
            app->next_action = milliseconds() + retry_delay(app);
            app->retries = 0;
            if (app->pending_request.payload_len)
                log_line(app, "link sequence %u sent (%zu bytes up, %s profile)",
                         app->sequence,
                         fragment_data_bytes(&app->pending_request),
                         profile_name(app->profile));
        }
    } else if (app->client_state == CLIENT_WAIT_RESPONSE &&
               now >= app->next_action) {
        ++app->profile_attempts;
        app->clean_transactions = 0;
        ++app->retries_sent;
        if (app->profile > MODEM_PROFILE_SAFE) {
            begin_profile_change(
                app, (enum modem_profile)(app->profile - 1), 1,
                "missed audio response");
        } else if (++app->retries > 4u) {
            reset_client(app, "audio response timed out");
        } else {
            log_line(app, "re-sending link sequence %u (%u/4)",
                     app->sequence, app->retries);
            (void)send_frame(app, &app->pending_request, 0);
            app->next_action = milliseconds() + retry_delay(app);
        }
    } else if (app->client_state == CLIENT_WAIT_PROFILE &&
               now >= app->next_action) {
        if (++app->retries > 4u) {
            reset_client(app, "profile change was not acknowledged");
        } else {
            log_line(app, "re-sending %s profile request (%u/4)",
                     profile_name(app->profile_target), app->retries);
            send_profile_control(app, app->profile_target, 0, app->sequence,
                                 0);
        }
    }
}

static void gateway_tick(app_t *app, uint64_t now)
{
    if (app->gateway_state == GATEWAY_WAIT_CONFIRM && now >= app->next_action) {
        if (++app->retries > 4u)
            reset_gateway(app, "client did not confirm the offer");
        else {
            log_line(app, "re-sending connection offer (%u/4)", app->retries);
            send_offer(app);
        }
    } else if (app->gateway_state == GATEWAY_CONNECTED &&
               now - app->last_receive > LINK_TIMEOUT_MS) {
        reset_gateway(app, "client heartbeat timed out");
    }
}

static void maybe_log_rate(app_t *app, uint64_t now)
{
    uint64_t elapsed, sent, received, upload, download;
    int connected = app->options.gateway
                        ? app->gateway_state == GATEWAY_CONNECTED
                        : app->client_state == CLIENT_READY ||
                              app->client_state == CLIENT_WAIT_RESPONSE ||
                              app->client_state == CLIENT_WAIT_PROFILE;

    if (!connected || now - app->last_rate < RATE_INTERVAL_MS)
        return;
    elapsed = now - app->last_rate;
    sent = app->bytes_sent - app->rate_bytes_sent;
    received = app->bytes_received - app->rate_bytes_received;
    upload = app->options.gateway ? received : sent;
    download = app->options.gateway ? sent : received;
    if (app->options.gateway) {
        log_line(app,
                 "throughput: upload %.1f B/s, download %.1f B/s; %s profile (SF%u/%zu bytes)",
                 upload * 1000.0 / elapsed, download * 1000.0 / elapsed,
                 profile_name(app->profile),
                 modem_profile_spreading_factor(app->profile),
                 modem_profile_payload_limit(app->profile));
    } else {
        double success = app->profile_attempts
                             ? app->profile_successes * 100.0 /
                                   app->profile_attempts
                             : 100.0;
        log_line(app,
                 "throughput: upload %.1f B/s, download %.1f B/s; %s profile, %.0f%% acknowledged, clean streak %u/%u",
                 upload * 1000.0 / elapsed, download * 1000.0 / elapsed,
                 profile_name(app->profile), success,
                 app->clean_transactions, PROMOTE_CLEAN_TRANSACTIONS);
    }
    app->last_rate = now;
    app->rate_bytes_sent = app->bytes_sent;
    app->rate_bytes_received = app->bytes_received;
}

static void maybe_log_stats(app_t *app, uint64_t now)
{
    if (now - app->last_stats < 30000u)
        return;
    app->last_stats = now;
    if (app->packets_from_tun || app->packets_to_tun || app->dropped_packets ||
        app->retries_sent)
        log_line(app,
                 "traffic: %llu packets read, %llu delivered, %llu retries, "
                 "%llu dropped",
                 (unsigned long long)app->packets_from_tun,
                 (unsigned long long)app->packets_to_tun,
                 (unsigned long long)app->retries_sent,
                 (unsigned long long)app->dropped_packets);
}

#ifdef __linux__
static int user_audio_device(const char *name)
{
    return !name || strcmp(name, "default") == 0 ||
           strcmp(name, "pulse") == 0;
}

static int sudo_identity(uid_t *uid, gid_t *gid, struct passwd **account)
{
    const char *uid_text = getenv("SUDO_UID");
    const char *gid_text = getenv("SUDO_GID");
    char *end;
    unsigned long value;

    if (geteuid() != 0 || !uid_text || !gid_text)
        return 0;
    errno = 0;
    value = strtoul(uid_text, &end, 10);
    if (errno || !*uid_text || *end || (uid_t)value != value || value == 0)
        return 0;
    *uid = (uid_t)value;
    errno = 0;
    value = strtoul(gid_text, &end, 10);
    if (errno || !*gid_text || *end || (gid_t)value != value)
        return 0;
    *gid = (gid_t)value;
    *account = getpwuid(*uid);
    return *account != NULL;
}

static int audio_open_for_app(app_t *app, char *err, size_t err_size)
{
    uid_t uid;
    gid_t gid;
    struct passwd *account;
    char runtime[128], server[160], cookie[512];
    int result, saved_errno;

    if ((!user_audio_device(app->options.input_device) &&
         !user_audio_device(app->options.output_device)) ||
        !sudo_identity(&uid, &gid, &account))
        return audio_open(&app->audio, app->options.input_device,
                          app->options.output_device, err, err_size);

    snprintf(runtime, sizeof(runtime), "/run/user/%lu", (unsigned long)uid);
    snprintf(server, sizeof(server), "unix:%s/pulse/native", runtime);
    snprintf(cookie, sizeof(cookie), "%s/.config/pulse/cookie",
             account->pw_dir);
    (void)setenv("HOME", account->pw_dir, 1);
    (void)setenv("USER", account->pw_name, 1);
    (void)setenv("LOGNAME", account->pw_name, 1);
    (void)setenv("XDG_RUNTIME_DIR", runtime, 1);
    (void)setenv("PULSE_SERVER", server, 1);
    if (access(cookie, R_OK) == 0)
        (void)setenv("PULSE_COOKIE", cookie, 1);

    log_line(app, "opening desktop audio as %s (uid %lu)", account->pw_name,
             (unsigned long)uid);
    if (setegid(gid) != 0) {
        saved_errno = errno;
        if (err && err_size)
            snprintf(err, err_size, "cannot assume invoking user's audio identity: %s",
                     strerror(saved_errno));
        return -1;
    }
    if (seteuid(uid) != 0) {
        saved_errno = errno;
        if (setegid(0) != 0) {
            if (err && err_size)
                snprintf(err, err_size,
                         "cannot assume invoking user's audio identity or restore group privileges");
        } else if (err && err_size) {
            snprintf(err, err_size,
                     "cannot assume invoking user's audio identity: %s",
                     strerror(saved_errno));
        }
        return -1;
    }
    result = audio_open(&app->audio, app->options.input_device,
                        app->options.output_device, err, err_size);
    saved_errno = errno;
    if (seteuid(0) != 0 || setegid(0) != 0) {
        if (app->audio) {
            audio_close(app->audio);
            app->audio = NULL;
        }
        if (err && err_size)
            snprintf(err, err_size, "cannot restore administrative privileges");
        return -1;
    }
    errno = saved_errno;
    return result;
}
#else
static int audio_open_for_app(app_t *app, char *err, size_t err_size)
{
    return audio_open(&app->audio, app->options.input_device,
                      app->options.output_device, err, err_size);
}
#endif

static int run_app(const options_t *options)
{
    app_t app;
    char error[256];
    struct sigaction action;
    size_t base_burst;

    memset(&app, 0, sizeof(app));
    app.options = *options;
    memset(&action, 0, sizeof(action));
    action.sa_handler = handle_signal;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
    signal(SIGPIPE, SIG_IGN);
    packet_queue_init(&app.outbound);
    packet_sender_init(&app.sender);
    packet_receiver_init(&app.receiver);
    app.bootstrap_decoder =
        modem_decoder_create(MODEM_MIN_HZ, MODEM_BOOTSTRAP_MAX_HZ);
    app.link_decoder = modem_decoder_create(options->low_hz, options->high_hz);
    if (!app.bootstrap_decoder || !app.link_decoder) {
        fprintf(stderr, "cannot allocate modem decoder\n");
        goto fail;
    }
    if (audio_open_for_app(&app, error, sizeof(error)) != 0) {
        fprintf(stderr, "%s\n", error);
        goto fail;
    }
    log_line(&app, "audio input:  %s", audio_input_name(app.audio));
    log_line(&app, "audio output: %s", audio_output_name(app.audio));
    log_line(&app,
             "audio format: mono 48 kHz; guarded-bin, convolutionally coded CSS; bootstrap/control SF10 at 2000-12000 Hz");
    log_line(&app,
             "data profiles: safe SF10/16 bytes -> robust SF9/32 -> balanced SF8/64 -> fast SF7/96");
    base_burst = modem_burst_samples(MODEM_MIN_HZ,
                                     MODEM_BOOTSTRAP_MAX_HZ,
                                     MODEM_PROFILE_SAFE);
    log_line(&app,
             "base profile timing: bursts up to %.1f s; retry window %.1f s",
             (double)base_burst / AUDIO_SAMPLE_RATE,
             bootstrap_retry_delay() / 1000.0);
    log_line(&app,
             "half-duplex timing: %u ms turnaround; local transmit echo suppressed",
             TURNAROUND_MS);

    if (tunnel_open(&app.tunnel, options->gateway, options->configure,
                    error, sizeof(error)) != 0) {
        log_line(&app, "%s", error);
        goto fail;
    }
    log_line(&app, "%s interface: %s (%s, MTU 1280)",
#ifdef __APPLE__
             "utun",
#else
             "TUN",
#endif
             tunnel_name(app.tunnel),
             options->configure ?
                 (options->gateway ? "10.77.0.1/fd77::1, forwarding/NAT enabled"
                                   : "10.77.0.2/fd77::2, default routes installed")
                 : "automatic network configuration disabled");

    if (options->gateway) {
        reset_gateway(&app, NULL);
        log_line(&app, "listening for one audio client");
    } else {
        reset_client(&app, NULL);
        log_line(&app, "searching for an audio gateway");
    }
    app.last_stats = milliseconds();
    app.next_audio_status = app.last_stats + 5000u;
    while (!stopping) {
        struct timespec pause = {0, 10000000};
        uint64_t now;
        pump_audio(&app);
        pump_tunnel(&app);
        now = milliseconds();
        if (options->gateway)
            gateway_tick(&app, now);
        else
            client_tick(&app, now);
        maybe_log_audio_status(&app, now);
        maybe_log_rate(&app, now);
        maybe_log_stats(&app, now);
        nanosleep(&pause, NULL);
    }
    log_line(&app, "shutting down; restoring network configuration");
    tunnel_close(app.tunnel);
    audio_close(app.audio);
    modem_decoder_destroy(app.link_decoder);
    modem_decoder_destroy(app.bootstrap_decoder);
    return 0;

fail:
    tunnel_close(app.tunnel);
    audio_close(app.audio);
    modem_decoder_destroy(app.link_decoder);
    modem_decoder_destroy(app.bootstrap_decoder);
    return 1;
}

int main(int argc, char **argv)
{
    options_t options;
    int self_test;

    if (parse_options(argc, argv, &options, &self_test) != 0)
        return 2;
    if (self_test) {
        printf("testing packet fragmentation... ");
        fflush(stdout);
        if (packet_self_test() != 0)
            return 1;
        printf("ok\ntesting CSS codec... ");
        fflush(stdout);
        if (modem_self_test() != 0)
            return 1;
        printf("ok\nall self-tests passed\n");
        return 0;
    }
    return run_app(&options);
}
