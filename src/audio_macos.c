#include "audio.h"

#include <AudioToolbox/AudioToolbox.h>
#include <CoreAudio/CoreAudio.h>
#include <CoreFoundation/CoreFoundation.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

#define RING_N (AUDIO_SAMPLE_RATE * 48u)
#define BLOCK_N 256u
#define QUEUE_BUFFERS 3u

struct audio {
    AudioQueueRef input_queue;
    AudioQueueRef output_queue;
    pthread_mutex_t mutex;
    pthread_cond_t space;
    _Atomic int stop;
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
                      OSStatus status)
{
    if (err && size)
        snprintf(err, size, "%s (CoreAudio error %d)", operation, (int)status);
}

static void input_callback(void *context, AudioQueueRef queue,
                           AudioQueueBufferRef buffer,
                           const AudioTimeStamp *start_time,
                           UInt32 packet_count,
                           const AudioStreamPacketDescription *packets)
{
    audio_t *audio = context;
    const int16_t *pcm = buffer->mAudioData;
    size_t count = buffer->mAudioDataByteSize / sizeof(*pcm), i;
    (void)start_time;
    (void)packet_count;
    (void)packets;

    pthread_mutex_lock(&audio->mutex);
    for (i = 0; i < count; ++i) {
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
    if (!atomic_load_explicit(&audio->stop, memory_order_relaxed)) {
        buffer->mAudioDataByteSize = BLOCK_N * sizeof(int16_t);
        AudioQueueEnqueueBuffer(queue, buffer, 0, NULL);
    }
}

static void output_callback(void *context, AudioQueueRef queue,
                            AudioQueueBufferRef buffer)
{
    audio_t *audio = context;
    int16_t *pcm = buffer->mAudioData;
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
    buffer->mAudioDataByteSize = BLOCK_N * sizeof(*pcm);
    if (!atomic_load_explicit(&audio->stop, memory_order_relaxed))
        AudioQueueEnqueueBuffer(queue, buffer, 0, NULL);
}

static void copy_cf_name(CFStringRef name, char *out, size_t size)
{
    if (!name || !CFStringGetCString(name, out, (CFIndex)size,
                                     kCFStringEncodingUTF8))
        snprintf(out, size, "system default");
}

static void default_device_name(AudioObjectPropertySelector selector,
                                char *out, size_t size)
{
    AudioObjectPropertyAddress address = {
        selector, kAudioObjectPropertyScopeGlobal,
        kAudioObjectPropertyElementMain};
    AudioDeviceID device = kAudioObjectUnknown;
    UInt32 data_size = sizeof(device);
    CFStringRef name = NULL;

    if (AudioObjectGetPropertyData(kAudioObjectSystemObject, &address, 0, NULL,
                                   &data_size, &device) != noErr) {
        snprintf(out, size, "system default");
        return;
    }
    address.mSelector = kAudioObjectPropertyName;
    data_size = sizeof(name);
    if (AudioObjectGetPropertyData(device, &address, 0, NULL, &data_size,
                                   &name) == noErr) {
        copy_cf_name(name, out, size);
        if (name)
            CFRelease(name);
    } else {
        snprintf(out, size, "system default");
    }
}

static OSStatus select_device(AudioQueueRef queue, const char *uid)
{
    CFStringRef value;
    OSStatus status;

    if (!uid)
        return noErr;
    value = CFStringCreateWithCString(NULL, uid, kCFStringEncodingUTF8);
    if (!value)
        return kAudio_ParamError;
    status = AudioQueueSetProperty(queue, kAudioQueueProperty_CurrentDevice,
                                   &value, sizeof(value));
    CFRelease(value);
    return status;
}

int audio_open(audio_t **out, const char *input_device,
               const char *output_device, char *err, size_t err_size)
{
    audio_t *audio;
    AudioStreamBasicDescription format;
    OSStatus status;
    unsigned i;

    if (!out)
        return -1;
    *out = NULL;
    audio = calloc(1, sizeof(*audio));
    if (!audio)
        return -1;
    pthread_mutex_init(&audio->mutex, NULL);
    pthread_cond_init(&audio->space, NULL);
    memset(&format, 0, sizeof(format));
    format.mSampleRate = AUDIO_SAMPLE_RATE;
    format.mFormatID = kAudioFormatLinearPCM;
    format.mFormatFlags = kLinearPCMFormatFlagIsSignedInteger |
                          kAudioFormatFlagIsPacked;
    format.mBytesPerPacket = sizeof(int16_t);
    format.mFramesPerPacket = 1;
    format.mBytesPerFrame = sizeof(int16_t);
    format.mChannelsPerFrame = 1;
    format.mBitsPerChannel = 16;

    status = AudioQueueNewInput(&format, input_callback, audio, NULL, NULL, 0,
                                &audio->input_queue);
    if (status == noErr)
        status = AudioQueueNewOutput(&format, output_callback, audio, NULL, NULL,
                                     0, &audio->output_queue);
    if (status == noErr)
        status = select_device(audio->input_queue, input_device);
    if (status == noErr)
        status = select_device(audio->output_queue, output_device);
    if (status != noErr) {
        set_error(err, err_size, "cannot open 48 kHz CoreAudio devices", status);
        audio_close(audio);
        return -1;
    }
    for (i = 0; i < QUEUE_BUFFERS; ++i) {
        AudioQueueBufferRef buffer;
        status = AudioQueueAllocateBuffer(audio->input_queue,
                                          BLOCK_N * sizeof(int16_t), &buffer);
        if (status == noErr) {
            buffer->mAudioDataByteSize = BLOCK_N * sizeof(int16_t);
            status = AudioQueueEnqueueBuffer(audio->input_queue, buffer, 0, NULL);
        }
        if (status != noErr)
            break;
    }
    for (i = 0; i < QUEUE_BUFFERS && status == noErr; ++i) {
        AudioQueueBufferRef buffer;
        status = AudioQueueAllocateBuffer(audio->output_queue,
                                          BLOCK_N * sizeof(int16_t), &buffer);
        if (status == noErr) {
            memset(buffer->mAudioData, 0, BLOCK_N * sizeof(int16_t));
            buffer->mAudioDataByteSize = BLOCK_N * sizeof(int16_t);
            status = AudioQueueEnqueueBuffer(audio->output_queue, buffer, 0,
                                             NULL);
        }
    }
    if (status == noErr)
        status = AudioQueueStart(audio->input_queue, NULL);
    if (status == noErr)
        status = AudioQueueStart(audio->output_queue, NULL);
    if (status != noErr) {
        set_error(err, err_size, "cannot start CoreAudio devices", status);
        audio_close(audio);
        return -1;
    }
    if (input_device)
        snprintf(audio->input_name, sizeof(audio->input_name), "%s (CoreAudio UID)",
                 input_device);
    else
        default_device_name(kAudioHardwarePropertyDefaultInputDevice,
                            audio->input_name, sizeof(audio->input_name));
    if (output_device)
        snprintf(audio->output_name, sizeof(audio->output_name),
                 "%s (CoreAudio UID)", output_device);
    else
        default_device_name(kAudioHardwarePropertyDefaultOutputDevice,
                            audio->output_name, sizeof(audio->output_name));
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
        if (err && err_size)
            snprintf(err, err_size, "invalid audio burst");
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
    return position == count ? 0 : -1;
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
    if (audio->input_queue) {
        AudioQueueStop(audio->input_queue, true);
        AudioQueueDispose(audio->input_queue, true);
    }
    if (audio->output_queue) {
        AudioQueueStop(audio->output_queue, true);
        AudioQueueDispose(audio->output_queue, true);
    }
    pthread_cond_destroy(&audio->space);
    pthread_mutex_destroy(&audio->mutex);
    free(audio);
}
