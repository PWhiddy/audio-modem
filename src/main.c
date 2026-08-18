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
#define SEARCH_MS 3000u
#define IDLE_POLL_MS 1000u
#define LINK_TIMEOUT_MS 12000u
#define TURNAROUND_MS 150u
#define OUTPUT_TAIL_MS 100u

enum client_state { CLIENT_SEARCH, CLIENT_WAIT_READY, CLIENT_READY,
                    CLIENT_WAIT_RESPONSE };
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
    double input_square_sum;
    float input_peak;
    uint64_t input_samples;
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
        *low + 1500u <= *high)
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
    options->high_hz = MODEM_MAX_HZ;
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
                fprintf(stderr, "invalid band; use LOW:HIGH within 2000:12000\n");
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
    unsigned high = bootstrap ? MODEM_MAX_HZ : app->selected_high;

    if (modem_encode(frame, low, high, &samples, &count) != 0) {
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
                                       app->selected_high);
    /* A complete request and response, plus device and room latency. */
    return 1000u + (uint64_t)burst * 2000u / AUDIO_SAMPLE_RATE;
}

static int set_link_band(app_t *app, unsigned low, unsigned high)
{
    if (modem_decoder_set_band(app->link_decoder, low, high) != 0)
        return -1;
    app->selected_low = low;
    app->selected_high = high;
    app->link_decoder_enabled = 1;
    app->link_matches_bootstrap =
        low == MODEM_MIN_HZ && high == MODEM_MAX_HZ;
    return 0;
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
        app->next_action = milliseconds() + SEARCH_MS;
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
        app->next_action = milliseconds() + SEARCH_MS;
}

static void send_confirm(app_t *app)
{
    modem_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.type = MODEM_CONFIRM;
    frame.session = app->session;
    frame.band_low = (uint16_t)app->selected_low;
    frame.band_high = (uint16_t)app->selected_high;
    if (send_frame(app, &frame, 0) == 0)
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
    (void)send_frame(app, &frame, 0);
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

static void client_frame(app_t *app, const modem_frame_t *frame)
{
    uint64_t now = milliseconds();

    if (frame->session != app->session)
        return;
    if (app->client_state == CLIENT_SEARCH && frame->type == MODEM_OFFER) {
        if (frame->band_low < app->options.low_hz ||
            frame->band_high > app->options.high_hz ||
            frame->band_low + 1500u > frame->band_high ||
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
        app->next_action = now;
        app->last_receive = now;
        log_line(app, "link up (session %08x, QPSK OFDM, %u-%u Hz)",
                 app->session, app->selected_low, app->selected_high);
        return;
    }
    if (app->client_state == CLIENT_WAIT_RESPONSE &&
        frame->type == MODEM_RESPONSE && frame->ack == app->sequence) {
        packet_sender_commit(&app->sender);
        deliver_fragment(app, frame);
        if (++app->sequence == 0)
            app->sequence = 1;
        app->client_state = CLIENT_READY;
        app->last_receive = now;
        app->retries = 0;
        app->next_action = (app->sender.active || app->outbound.count ||
                            (frame->flags & FLAG_MORE))
                               ? now
                               : now + IDLE_POLL_MS;
    }
}

static void gateway_frame(app_t *app, const modem_frame_t *frame)
{
    uint64_t now = milliseconds();

    if (frame->type == MODEM_HELLO &&
        (app->gateway_state != GATEWAY_CONNECTED ||
         frame->session == app->session)) {
        unsigned low = frame->band_low > app->options.low_hz
                           ? frame->band_low : app->options.low_hz;
        unsigned high = frame->band_high < app->options.high_hz
                            ? frame->band_high : app->options.high_hz;
        if (low + 1500u > high)
            return;
        if (app->gateway_state == GATEWAY_LISTEN ||
            frame->session != app->session) {
            app->session = frame->session;
            if (set_link_band(app, low, high) != 0)
                return;
            app->gateway_state = GATEWAY_WAIT_CONFIRM;
            app->retries = 0;
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
        packet_receiver_init(&app->receiver);
        log_line(app, "link up (session %08x, QPSK OFDM, %u-%u Hz)",
                 app->session, app->selected_low, app->selected_high);
        return;
    }
    if (app->gateway_state == GATEWAY_CONNECTED &&
        frame->type == MODEM_CONFIRM) {
        send_ready(app);
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
        packet_sender_commit(&app->sender);
    app->response_fragment_pending = 0;
    deliver_fragment(app, frame);
    memset(&app->cached_response, 0, sizeof(app->cached_response));
    app->cached_response.type = MODEM_RESPONSE;
    app->cached_response.session = app->session;
    app->cached_response.seq = frame->seq;
    app->cached_response.ack = frame->seq;
    app->cached_response.payload_len = (uint16_t)packet_sender_fragment(
        &app->sender, &app->outbound, app->cached_response.payload);
    app->response_fragment_pending = app->cached_response.payload_len != 0;
    if (app->response_fragment_pending || app->outbound.count)
        app->cached_response.flags |= FLAG_MORE;
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
        if (milliseconds() < app->ignore_input_until)
            continue;
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
    modem_activity_t activity;
    int connecting;

    if (now < app->next_audio_status)
        return;
    app->next_audio_status = now + 5000u;
    modem_decoder_take_activity(app->bootstrap_decoder, &activity);
    connecting = app->options.gateway
                     ? app->gateway_state != GATEWAY_CONNECTED
                     : app->client_state != CLIENT_READY &&
                           app->client_state != CLIENT_WAIT_RESPONSE;
    if (connecting) {
        if (app->input_samples) {
            double rms = sqrt(app->input_square_sum / app->input_samples);
            double rms_db = 20.0 * log10(rms > 1.0e-6 ? rms : 1.0e-6);
            double peak_db = 20.0 * log10(app->input_peak > 1.0e-6f
                                              ? app->input_peak
                                              : 1.0e-6f);
            log_line(app,
                     "input level: %.0f dBFS RMS, %.0f dBFS peak; best modem sync %.2f%s",
                     rms_db, peak_db, activity.peak_sync,
                     activity.rejected ? "; candidate frame rejected" : "");
        } else {
            log_line(app, "audio input produced no samples");
        }
    }
    app->input_square_sum = 0.0;
    app->input_peak = 0.0f;
    app->input_samples = 0;
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
        memset(&app->pending_request, 0, sizeof(app->pending_request));
        app->pending_request.type = MODEM_REQUEST;
        app->pending_request.session = app->session;
        app->pending_request.seq = app->sequence;
        app->pending_request.payload_len = (uint16_t)packet_sender_fragment(
            &app->sender, &app->outbound, app->pending_request.payload);
        if (app->sender.active || app->outbound.count)
            app->pending_request.flags |= FLAG_MORE;
        if (send_frame(app, &app->pending_request, 0) == 0) {
            app->client_state = CLIENT_WAIT_RESPONSE;
            app->next_action = now + retry_delay(app);
            app->retries = 0;
        }
    } else if (app->client_state == CLIENT_WAIT_RESPONSE &&
               now >= app->next_action) {
        if (++app->retries > 4u) {
            reset_client(app, "audio response timed out");
        } else {
            log_line(app, "re-sending link sequence %u (%u/4)",
                     app->sequence, app->retries);
            (void)send_frame(app, &app->pending_request, 0);
            app->next_action = now + retry_delay(app);
            ++app->retries_sent;
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
    app.bootstrap_decoder = modem_decoder_create(MODEM_MIN_HZ, MODEM_MAX_HZ);
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
             "audio format: mono 48 kHz; QPSK OFDM; bootstrap band 2000-12000 Hz");
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
        printf("ok\ntesting QPSK OFDM codec... ");
        fflush(stdout);
        if (modem_self_test() != 0)
            return 1;
        printf("ok\nall self-tests passed\n");
        return 0;
    }
    return run_app(&options);
}
