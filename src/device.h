#ifndef SOUNDLINK_DEVICE_H
#define SOUNDLINK_DEVICE_H

#include "framework.h"
#include "miniaudio.h"
#include "generic.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
constexpr ma_uint32 SAMPLE_RATE = 48000;
constexpr ma_uint32 CAPACITY    = SAMPLE_RATE;      // 1 second.
constexpr ma_uint32 TARGET_FILL = SAMPLE_RATE / 4;  // 250 ms.
constexpr ma_uint32 SLEEP_THRESHOLD = SAMPLE_RATE / 5;

namespace soundlink {

struct frame_timing {
    uint64_t sample = 0;  // 48 kHz capture sample position.
    u8 slot_number = 0;
    frame_timing& operator+=(unsigned n) {
        sample += 24000 * n;
        slot_number = (slot_number + n) % 8;
        return *this;
    }
    frame_timing operator+(unsigned n) const {
        return {
            sample + 24000 * n,
            static_cast<u8>((slot_number + n) % 8)
        };
    }
};
struct waterfall {
    static const size_t sample_count = 2048;

    void request(const sample_request<float>& req) {
        pending = req;
        filled = 0;
    }

    void append(const float* samples, size_t frames) {
        if (!pending.ready) return;

        const size_t count = std::min(frames, sample_count - filled);
        std::copy_n(samples, count, pending.points + filled);
        filled += count;
        if (filled < sample_count) return;

        const auto req = pending;
        pending = {};
        *req.written = sample_count;
        req.ready();
    }

    ~waterfall() {
        if (!pending.ready) return;
        *pending.written = 0;
        pending.ready();
    }

private:
    sample_request<float> pending{};
    size_t filled = 0;
};
inline void audio_callback(ma_device* device, void* output, const void* input, ma_uint32 count);

// One worker reads mic / writes speaker; the callback uses the opposite ends.
struct audio_device {
    void request_waterfall(sample_request<float> req) { plot.request(req); }


    audio_device(u8 mask, const char* recording_path = nullptr) : slt_mask(mask) {
        if(recording_path){
            recorder = new mic_recorder(recording_path);
        }
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
        delete recorder;
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
    uint64_t delivered2rx() const {
        return delivered;
    }
    void play(){
        m = mode::play;
        next += 1;
        }
    void arm() {
        m = mode::armed;
    }

    void mix_audio(const std::vector<tx_channel*>& tx) {
        bool mix = false;
        size_t count = ma_pcm_rb_available_write(&speaker);
        if (m == mode::armed && played == next.sample) {
            m = mode::play;
            next += 1;
        }
        switch (m) {
        case mode::wait:
            count = std::min<size_t>(count, TARGET_FILL/5);
            break;
        case mode::armed:
            count = std::min<uint64_t>(count, next.sample - played);
            break;
        case mode::play:
            adjust(tx);
        case mode::close:{
            size_t queued = ma_pcm_rb_available_read(&speaker);
            if (queued >= TARGET_FILL) return;
            count = std::min<size_t>(ma_pcm_rb_available_write(&speaker), TARGET_FILL - queued);
            mix = true;
            break;
        }
        case mode::closed:
            return;
        }
        if(m == mode::close)
            count = std::min<uint64_t>(count, end.sample - played);

        float gain = 0.0;
        if (m == mode::play || m == mode::close){
            for (auto* channel : tx)
                count = std::min(count, channel->available());
            gain = 1.0f/float(tx.size());
        }

        while (count > 0) {
            ma_uint32 frames = static_cast<ma_uint32>(count);
            void* memory = nullptr;
            ma_pcm_rb_acquire_write(&speaker, &frames, &memory);

            float* output = static_cast<float*>(memory);
            std::fill_n(output, frames, 0.0f);

            if (mix) {
                for (auto* channel : tx) {
                    ma_mix_pcm_frames_f32(output, channel->samples(), frames, 1, gain);
                    channel->consume(frames);
                }
            }
            ma_pcm_rb_commit_write(&speaker, frames);
            count -= frames;
            played += frames;
            if (m == mode::play || m == mode::close) {
                while (played >= next.sample) next += 1;
            }
            if (m == mode::close && played == end.sample){
                this->stop();
                m = mode::closed;
            }
        }
    }

    inline void read_mic(const std::vector<rx_channel*>& rx) {
        if (rx.empty()) return;
        size_t count = ma_pcm_rb_available_read(&mic);

        for (auto* channel : rx)
            count = std::min(count, channel->writable());

        while (count > 0) {
            ma_uint32 frames = static_cast<ma_uint32>(count);
            void* memory = nullptr;
            ma_pcm_rb_acquire_read(&mic, &frames, &memory);

            for (auto* channel : rx) {
                std::memcpy(channel->samples(), memory, frames * sizeof(float));
                channel->written(frames);
            }
            plot.append((const float*)memory, frames);
            ma_pcm_rb_commit_read(&mic, frames);
            count -= frames;
            delivered += frames;
        }
    }
    u8 calibrate(frame_timing* report) {
        if (report->sample > played) {
            next = *report;
            return next.slot_number;
        }
        uint64_t passed_slots = (played - report->sample) / 24000;
        u8 next_slot = (report->slot_number + passed_slots % 8 + 1) & 7u;

        next.sample = report->sample + (passed_slots + 1) * 24000;
        next.slot_number = next_slot;
        return next_slot;
    }
    u8 update_mask(u8 _mask){
        slt_mask = _mask;
        while (!((slt_mask >> (7 - next.slot_number)) & 1u)) {
            next += 1;
        }
        m = mode::armed;
        return next.slot_number;
    }
    void set_adj(int16_t adj){
        t_adj += adj;
    }
    int16_t get_adj() const {
        return t_adj;
    }
    void stop_after(unsigned count) {
        end = next + count;
        m = mode::close;
    }
    bool is_done() const {
        return m == mode::closed;
    }
    void adjust(const std::vector<tx_channel*>& tx) {
        u8 current_slot = (next.slot_number + 7) & 7u;
        bool silent = ((slt_mask >> (7 - current_slot)) & 1u) == 0;
        uint64_t remaining = next.sample - played;

        if (!silent) {
            return;
        }
        if (remaining > 20000 || !t_adj || last_adjusted_slot == current_slot)
            return;

        size_t count = std::min<int>(240, std::abs(int(t_adj)));

        if (t_adj > 0) {
            count = std::min<size_t>(count, ma_pcm_rb_available_write(&speaker));
            if (count == 0) return;
            while (count > 0) {
                ma_uint32 frames = static_cast<ma_uint32>(count);
                void* memory = nullptr;
                ma_pcm_rb_acquire_write(&speaker, &frames, &memory);

                std::fill_n(static_cast<float*>(memory), frames, 0.0f);

                ma_pcm_rb_commit_write(&speaker, frames);
                played += frames;
                next.sample += frames;
                t_adj -= frames;
                count -= frames;
            }
        } else {
            count = std::min<uint64_t>(count, next.sample - played);
            for (auto* channel : tx)
                count = std::min(count, channel->available());

            for (auto* channel : tx)
                channel->consume(count);

            next.sample -= count;
            t_adj += count;
        }
        last_adjusted_slot = current_slot;
    }

    ma_pcm_rb mic{};
    ma_pcm_rb speaker{};
    mic_recorder* recorder = nullptr;
private:
    u8 slt_mask;
    ma_device device{};
    frame_timing next, end;
    uint64_t played = 0;
    uint64_t delivered = 0;
    int16_t t_adj = 0;
    u8 last_adjusted_slot = 10;
    enum class mode : u8 {wait, armed, play, close, closed};
    mode m = mode::wait;
    waterfall plot;
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
    if (self.recorder)
        self.recorder->push(static_cast<const float*>(input), count);

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
