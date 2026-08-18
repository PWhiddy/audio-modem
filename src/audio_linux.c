#include "audio.h"

#include <dlfcn.h>
#include <pthread.h>
#include <stdint.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RING_N (AUDIO_SAMPLE_RATE * 40u)
#define BLOCK_N 256u
#define PCM_PLAYBACK 0
#define PCM_CAPTURE 1
#define PCM_FORMAT_S16_LE 2
#define PCM_ACCESS_RW_INTERLEAVED 3

typedef int (*pcm_open_fn)(void **, const char *, int, int);
typedef int (*pcm_close_fn)(void *);
typedef int (*pcm_set_params_fn)(void *, int, int, unsigned, unsigned, int,
                                 unsigned);
typedef long (*pcm_readi_fn)(void *, void *, unsigned long);
typedef long (*pcm_writei_fn)(void *, const void *, unsigned long);
typedef int (*pcm_recover_fn)(void *, int, int);
typedef const char *(*strerror_fn)(int);
typedef const char *(*pcm_name_fn)(void *);
typedef int (*pcm_info_malloc_fn)(void **);
typedef void (*pcm_info_free_fn)(void *);
typedef int (*pcm_info_fn)(void *, void *);
typedef const char *(*pcm_info_name_fn)(const void *);
typedef int (*pcm_info_card_fn)(const void *);
typedef unsigned (*pcm_info_device_fn)(const void *);
typedef int (*card_name_fn)(int, char **);

struct audio {
    void *library;
    void *capture;
    void *playback;
    pcm_close_fn pcm_close;
    pcm_readi_fn pcm_readi;
    pcm_writei_fn pcm_writei;
    pcm_recover_fn pcm_recover;
    strerror_fn error_string;
    pthread_t capture_thread;
    pthread_t playback_thread;
    int capture_started;
    int playback_started;
    _Atomic int stop;
    pthread_mutex_t mutex;
    pthread_cond_t space;
    float rx[RING_N];
    size_t rx_head;
    size_t rx_count;
    float tx[RING_N];
    size_t tx_head;
    size_t tx_count;
    char input_name[256];
    char output_name[256];
};

static void set_error(char *err, size_t size, const char *operation,
                      const char *detail)
{
    if (err && size)
        snprintf(err, size, "%s: %s", operation, detail ? detail : "failed");
}

static void *load_symbol(void *library, const char *name)
{
    return dlsym(library, name);
}

static void describe_pcm(char *out, size_t out_size, const char *configured,
                         void *pcm, pcm_name_fn pcm_name,
                         pcm_info_malloc_fn info_malloc,
                         pcm_info_free_fn info_free, pcm_info_fn get_info,
                         pcm_info_name_fn info_name,
                         pcm_info_card_fn info_card,
                         pcm_info_device_fn info_device,
                         card_name_fn card_name)
{
    void *info = NULL;
    const char *pcm_description = NULL;
    char *card_description = NULL;
    int card = -1;
    unsigned device = 0;

    if (info_malloc && info_free && get_info && info_name && info_card &&
        info_device && info_malloc(&info) == 0) {
        if (get_info(pcm, info) == 0) {
            pcm_description = info_name(info);
            card = info_card(info);
            device = info_device(info);
            if (card >= 0 && card_name)
                (void)card_name(card, &card_description);
        }
    }
    if (pcm_description && *pcm_description) {
        if (card >= 0 && card_description && *card_description &&
            strcmp(card_description, pcm_description) != 0)
            snprintf(out, out_size, "%s -> %s / %s (ALSA card %d, device %u)",
                     configured, card_description, pcm_description, card,
                     device);
        else if (card >= 0)
            snprintf(out, out_size, "%s -> %s (ALSA card %d, device %u)",
                     configured, pcm_description, card, device);
        else
            snprintf(out, out_size, "%s -> %s (ALSA virtual PCM)", configured,
                     pcm_description);
    } else {
        const char *identifier = pcm_name(pcm);
        snprintf(out, out_size, "%s (ALSA%s%s)",
                 identifier ? identifier : configured,
                 strcmp(configured, "default") == 0 ? " default route" : "",
                 strcmp(configured, "default") == 0
                     ? "; backing device not exposed"
                     : "");
    }
    free(card_description);
    if (info)
        info_free(info);
}

static void *capture_main(void *argument)
{
    audio_t *audio = argument;
    int16_t pcm[BLOCK_N];

    while (!atomic_load_explicit(&audio->stop, memory_order_relaxed)) {
        long got = audio->pcm_readi(audio->capture, pcm, BLOCK_N);
        size_t i;
        if (got < 0) {
            audio->pcm_recover(audio->capture, (int)got, 1);
            continue;
        }
        pthread_mutex_lock(&audio->mutex);
        for (i = 0; i < (size_t)got; ++i) {
            size_t slot;
            if (audio->rx_count == RING_N) {
                audio->rx_head = (audio->rx_head + 1u) % RING_N;
                --audio->rx_count;
            }
            slot = (audio->rx_head + audio->rx_count) % RING_N;
            audio->rx[slot] = pcm[i] / 32768.0f;
            ++audio->rx_count;
        }
        pthread_mutex_unlock(&audio->mutex);
    }
    return NULL;
}

static void *playback_main(void *argument)
{
    audio_t *audio = argument;
    int16_t pcm[BLOCK_N];

    while (!atomic_load_explicit(&audio->stop, memory_order_relaxed)) {
        size_t i;
        pthread_mutex_lock(&audio->mutex);
        for (i = 0; i < BLOCK_N; ++i) {
            float sample = 0.0f;
            if (audio->tx_count) {
                sample = audio->tx[audio->tx_head];
                audio->tx_head = (audio->tx_head + 1u) % RING_N;
                --audio->tx_count;
            }
            if (sample > 1.0f)
                sample = 1.0f;
            if (sample < -1.0f)
                sample = -1.0f;
            pcm[i] = (int16_t)(sample * 32767.0f);
        }
        pthread_cond_broadcast(&audio->space);
        pthread_mutex_unlock(&audio->mutex);
        {
            size_t offset = 0;
            while (offset < BLOCK_N &&
                   !atomic_load_explicit(&audio->stop, memory_order_relaxed)) {
                long wrote = audio->pcm_writei(audio->playback, pcm + offset,
                                               BLOCK_N - offset);
                if (wrote < 0) {
                    audio->pcm_recover(audio->playback, (int)wrote, 1);
                    break;
                }
                offset += (size_t)wrote;
            }
        }
    }
    return NULL;
}

int audio_open(audio_t **out, const char *input_device,
               const char *output_device, char *err, size_t err_size)
{
    audio_t *audio;
    pcm_open_fn pcm_open;
    pcm_set_params_fn pcm_set_params;
    pcm_name_fn pcm_name;
    pcm_info_malloc_fn info_malloc;
    pcm_info_free_fn info_free;
    pcm_info_fn get_info;
    pcm_info_name_fn info_name;
    pcm_info_card_fn info_card;
    pcm_info_device_fn info_device;
    card_name_fn card_name;
    int status;

    if (!out)
        return -1;
    *out = NULL;
    audio = calloc(1, sizeof(*audio));
    if (!audio) {
        set_error(err, err_size, "audio", "out of memory");
        return -1;
    }
    pthread_mutex_init(&audio->mutex, NULL);
    pthread_cond_init(&audio->space, NULL);
    audio->library = dlopen("libasound.so.2", RTLD_NOW | RTLD_LOCAL);
    if (!audio->library) {
        set_error(err, err_size, "audio", "libasound.so.2 is not installed");
        audio_close(audio);
        return -1;
    }
#define LOAD(member, symbol)                                                   \
    do {                                                                       \
        *(void **)(&(member)) = load_symbol(audio->library, symbol);            \
        if (!(member)) {                                                       \
            set_error(err, err_size, "audio", "incompatible ALSA library");  \
            audio_close(audio);                                                \
            return -1;                                                         \
        }                                                                      \
    } while (0)
    LOAD(pcm_open, "snd_pcm_open");
    LOAD(audio->pcm_close, "snd_pcm_close");
    LOAD(pcm_set_params, "snd_pcm_set_params");
    LOAD(audio->pcm_readi, "snd_pcm_readi");
    LOAD(audio->pcm_writei, "snd_pcm_writei");
    LOAD(audio->pcm_recover, "snd_pcm_recover");
    LOAD(audio->error_string, "snd_strerror");
    LOAD(pcm_name, "snd_pcm_name");
#undef LOAD
#define LOAD_OPTIONAL(member, symbol)                                           \
    (*(void **)(&(member)) = load_symbol(audio->library, symbol))
    LOAD_OPTIONAL(info_malloc, "snd_pcm_info_malloc");
    LOAD_OPTIONAL(info_free, "snd_pcm_info_free");
    LOAD_OPTIONAL(get_info, "snd_pcm_info");
    LOAD_OPTIONAL(info_name, "snd_pcm_info_get_name");
    LOAD_OPTIONAL(info_card, "snd_pcm_info_get_card");
    LOAD_OPTIONAL(info_device, "snd_pcm_info_get_device");
    LOAD_OPTIONAL(card_name, "snd_card_get_name");
#undef LOAD_OPTIONAL

    if (!input_device)
        input_device = "default";
    if (!output_device)
        output_device = "default";
    status = pcm_open(&audio->capture, input_device, PCM_CAPTURE, 0);
    if (status < 0) {
        set_error(err, err_size, "cannot open audio input",
                  audio->error_string(status));
        audio_close(audio);
        return -1;
    }
    status = pcm_open(&audio->playback, output_device, PCM_PLAYBACK, 0);
    if (status < 0) {
        set_error(err, err_size, "cannot open audio output",
                  audio->error_string(status));
        audio_close(audio);
        return -1;
    }
    status = pcm_set_params(audio->capture, PCM_FORMAT_S16_LE,
                            PCM_ACCESS_RW_INTERLEAVED, 1, AUDIO_SAMPLE_RATE, 1,
                            40000);
    if (status >= 0)
        status = pcm_set_params(audio->playback, PCM_FORMAT_S16_LE,
                                PCM_ACCESS_RW_INTERLEAVED, 1, AUDIO_SAMPLE_RATE,
                                1, 40000);
    if (status < 0) {
        set_error(err, err_size, "cannot configure 48 kHz audio",
                  audio->error_string(status));
        audio_close(audio);
        return -1;
    }
    describe_pcm(audio->input_name, sizeof(audio->input_name), input_device,
                 audio->capture, pcm_name, info_malloc, info_free, get_info,
                 info_name, info_card, info_device, card_name);
    describe_pcm(audio->output_name, sizeof(audio->output_name), output_device,
                 audio->playback, pcm_name, info_malloc, info_free, get_info,
                 info_name, info_card, info_device, card_name);
    if (pthread_create(&audio->capture_thread, NULL, capture_main, audio) != 0) {
        set_error(err, err_size, "audio", "cannot start capture thread");
        audio_close(audio);
        return -1;
    }
    audio->capture_started = 1;
    if (pthread_create(&audio->playback_thread, NULL, playback_main, audio) != 0) {
        set_error(err, err_size, "audio", "cannot start playback thread");
        audio_close(audio);
        return -1;
    }
    audio->playback_started = 1;
    *out = audio;
    return 0;
}

size_t audio_read(audio_t *audio, float *samples, size_t count)
{
    size_t i;

    if (!audio || !samples)
        return 0;
    pthread_mutex_lock(&audio->mutex);
    if (count > audio->rx_count)
        count = audio->rx_count;
    for (i = 0; i < count; ++i) {
        samples[i] = audio->rx[audio->rx_head];
        audio->rx_head = (audio->rx_head + 1u) % RING_N;
    }
    audio->rx_count -= count;
    pthread_mutex_unlock(&audio->mutex);
    return count;
}

int audio_send(audio_t *audio, const float *samples, size_t count,
               char *err, size_t err_size)
{
    size_t position = 0;

    if (!audio || !samples || count > RING_N) {
        set_error(err, err_size, "audio send", "invalid burst");
        return -1;
    }
    pthread_mutex_lock(&audio->mutex);
    while (position < count &&
           !atomic_load_explicit(&audio->stop, memory_order_relaxed)) {
        size_t room = RING_N - audio->tx_count;
        size_t chunk = count - position;
        size_t i;
        if (!room) {
            pthread_cond_wait(&audio->space, &audio->mutex);
            continue;
        }
        if (chunk > room)
            chunk = room;
        for (i = 0; i < chunk; ++i) {
            size_t slot = (audio->tx_head + audio->tx_count) % RING_N;
            audio->tx[slot] = samples[position + i];
            ++audio->tx_count;
        }
        position += chunk;
    }
    pthread_mutex_unlock(&audio->mutex);
    if (position != count) {
        set_error(err, err_size, "audio send", "audio device stopped");
        return -1;
    }
    return 0;
}

const char *audio_input_name(const audio_t *audio)
{
    return audio ? audio->input_name : "unknown";
}

const char *audio_output_name(const audio_t *audio)
{
    return audio ? audio->output_name : "unknown";
}

void audio_close(audio_t *audio)
{
    if (!audio)
        return;
    atomic_store_explicit(&audio->stop, 1, memory_order_relaxed);
    pthread_cond_broadcast(&audio->space);
    if (audio->capture_started)
        pthread_join(audio->capture_thread, NULL);
    if (audio->playback_started)
        pthread_join(audio->playback_thread, NULL);
    if (audio->capture && audio->pcm_close)
        audio->pcm_close(audio->capture);
    if (audio->playback && audio->pcm_close)
        audio->pcm_close(audio->playback);
    if (audio->library)
        dlclose(audio->library);
    pthread_cond_destroy(&audio->space);
    pthread_mutex_destroy(&audio->mutex);
    free(audio);
}
