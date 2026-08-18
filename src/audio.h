#ifndef AUDIO_MODEM_AUDIO_H
#define AUDIO_MODEM_AUDIO_H

#include <stddef.h>

#define AUDIO_SAMPLE_RATE 48000u

typedef struct audio audio_t;

/* Device names may be NULL for the platform defaults. */
int audio_open(audio_t **out, const char *input_device,
               const char *output_device, char *err, size_t err_size);
size_t audio_read(audio_t *audio, float *samples, size_t count);
int audio_send(audio_t *audio, const float *samples, size_t count,
               char *err, size_t err_size);
const char *audio_input_name(const audio_t *audio);
const char *audio_output_name(const audio_t *audio);
void audio_close(audio_t *audio);

#endif
