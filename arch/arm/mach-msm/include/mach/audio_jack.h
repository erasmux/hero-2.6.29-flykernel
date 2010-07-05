/*
 * Definitions for audio jack.
 */
#ifndef AUDIO_JACK_H
#define AUDIO_JACK_H


struct audio_jack_platform_data {
	int gpio;
};

int is_audio_jack_pin_stable(void);

#endif
