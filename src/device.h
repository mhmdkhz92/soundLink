#ifndef SOUNDLINK_DEVICE_H
#define SOUNDLINK_DEVICE_H


#include "framework.h"
#include "miniaudio.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

constexpr ma_uint32 SAMPLE_RATE = 48000;
constexpr ma_uint32 CAPACITY    = SAMPLE_RATE;      // 1 second.
constexpr ma_uint32 TARGET_FILL = SAMPLE_RATE / 4;  // 250 ms.
constexpr ma_uint32 SLEEP_THRESHOLD = SAMPLE_RATE / 5;

namespace soundlink{

inline void audio_callback(ma_device* device, void* output, const void* input, ma_uint32 count);

// One worker reads mic / writes speaker; the callback uses the opposite ends.
struct audio_device {
    audio_device() {
        ma_pcm_rb_init(ma_format_f32, 1, CAPACITY, nullptr, nullptr, &mic);
        ma_pcm_rb_init(ma_format_f32, 1, CAPACITY, nullptr, nullptr, &speaker);

        ma_device_config config = ma_device_config_init(ma_device_type_duplex);
        config.capture.format = ma_format_f32;
        config.capture.channels = 1;
        config.playback.format = ma_format_f32;
        config.playback.channels = 1;
        config.sampleRate = SAMPLE_RATE;
        config.noPreSilencedOutputBuffer = MA_TRUE;
        config.dataCallback = audio_callback;
        config.pUserData = this;

        ma_device_init(nullptr, &config, &device);
    }

    ~audio_device() {
        ma_device_uninit(&device);
        ma_pcm_rb_uninit(&speaker);
        ma_pcm_rb_uninit(&mic);
    }

    // Prefill playback before starting. A shortage is fatal, not padded.
    void start() {
        if (ma_device_start(&device) != MA_SUCCESS)
            throw std::runtime_error("Cannot start audio device");
    }

    void stop() {
        if (ma_device_stop(&device) != MA_SUCCESS)
            throw std::runtime_error("Cannot stop audio device");
    }

    ma_pcm_rb mic{};
    ma_pcm_rb speaker{};

private:
    ma_device device{};
};

inline void audio_callback(ma_device* device, void* output, const void* input, ma_uint32 count) {
    if (count == 0) return;
    auto& self = *static_cast<audio_device*>(device->pUserData);

    if (!input || !output ||
        ma_pcm_rb_available_read(&self.speaker) < count ||
        ma_pcm_rb_available_write(&self.mic) < count) {
        std::fputs("Duplex audio buffer failure\n", stderr);
        std::_Exit(EXIT_FAILURE);
    }

    ma_uint32 copied = 0;
    while (copied < count) {
        ma_uint32 frames = count - copied;
        void* memory = nullptr;
        ma_pcm_rb_acquire_write(&self.mic, &frames, &memory);
        std::memcpy(memory, static_cast<const float*>(input) + copied, frames * sizeof(float));
        ma_pcm_rb_commit_write(&self.mic, frames);
        copied += frames;
    }

    copied = 0;
    while (copied < count) {
        ma_uint32 frames = count - copied;
        void* memory = nullptr;
        ma_pcm_rb_acquire_read(&self.speaker, &frames, &memory);
        std::memcpy(static_cast<float*>(output) + copied, memory, frames * sizeof(float));
        ma_pcm_rb_commit_read(&self.speaker, frames);
        copied += frames;
    }
}


} // namespace soundlink

#endif
