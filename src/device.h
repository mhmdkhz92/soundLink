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

namespace soundlink {

struct tx_channel {
    virtual void set_sltn(u8 n) = 0;
    virtual size_t available() = 0;
    virtual const float* samples() = 0;
    virtual void consume(size_t count) = 0;
    virtual void step() = 0;
    virtual bool run() = 0;
    virtual ~tx_channel() = default;
};
struct frame_timing;

struct rx_channel {
    virtual void set_mask(u8 mask) = 0;
    virtual size_t writable() = 0;
    virtual float* samples() = 0;
    virtual void written(size_t count) = 0;
    virtual void step() = 0;
    virtual bool get_stamp(frame_timing& ft) = 0;
    virtual void clear_stamp() = 0;

    virtual ~rx_channel() = default;
};

struct frame_timing {
    uint64_t sample = 0;  // 48 kHz capture sample position.
    u8 slot_number = 0;
};
inline void audio_callback(ma_device* device, void* output, const void* input, ma_uint32 count);

// One worker reads mic / writes speaker; the callback uses the opposite ends.
struct audio_device {
    audio_device(u8 mask) : slt_mask(mask) {
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
    void play(){
        m = mode::play;
        tx_stamp.sample = 24000;
        tx_stamp.slot_number = 1;
        }
    void arm() {
        m = mode::armed;
    }

    void mix_audio(const std::vector<tx_channel*>& tx) {
        bool mix = false;
        size_t count = ma_pcm_rb_available_write(&speaker);
        if (m == mode::armed && played == tx_stamp.sample) {
            m = mode::play;
            increment_stamp();
        }
        switch (m) {
        case mode::wait:
            count = std::min<size_t>(count, TARGET_FILL/5);
            break;
        case mode::armed:
            count = std::min<uint64_t>(count, tx_stamp.sample - played);
            break;
        case mode::play:
            adjust(tx);
            size_t queued = ma_pcm_rb_available_read(&speaker);
            if (queued >= TARGET_FILL) return;

            count = std::min<size_t>(
            ma_pcm_rb_available_write(&speaker), TARGET_FILL - queued);

            for (auto* channel : tx)
                count = std::min(count, channel->available());
            mix = true;
            break;
        }

        const float gain = mix ? 1.0f / float(tx.size()) : 0.0f;

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
            if (m == mode::play) {
                while (played >= tx_stamp.sample) increment_stamp();
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
            ma_pcm_rb_commit_read(&mic, frames);
            count -= frames;
        }
    }
    u8 next_opportunity(frame_timing* report) {
        if (report->sample > played) {
            tx_stamp = *report;
            return tx_stamp.slot_number;
        }
        uint64_t passed_slots = (played - report->sample) / 24000;
        u8 next = (report->slot_number + passed_slots % 8 + 1) & 7u;

        tx_stamp.sample = report->sample + (passed_slots + 1) * 24000;
        tx_stamp.slot_number = next;
        return next;
    }
    void set_adjustment(int16_t adj){
        t_adj += adj;
    }
    bool adjustment_complete() const {
        return t_adj == 0;
    }
    void stop_after_active_slots(unsigned count) {
        stop_sample = tx_stamp.sample;
        u8 slot_number = tx_stamp.slot_number;

        while (count > 0) {
            stop_sample += 24000;
            if ((slt_mask >> (7 - slot_number)) & 1u)
                --count;
            slot_number = (slot_number + 1) & 7u;
        }

        stop_scheduled = true;
    }
    bool stop_ready() {
        if (!stop_scheduled) return false;
        uint64_t consumed = played - ma_pcm_rb_available_read(&speaker);
        return consumed >= stop_sample;
    }
    void adjust(const std::vector<tx_channel*>& tx) {
        u8 current_slot = (tx_stamp.slot_number + 7) & 7u;
        bool silent = ((slt_mask >> (7 - current_slot)) & 1u) == 0;
        uint64_t remaining = tx_stamp.sample - played;

        if (!silent) {
            adjusted = false;
            return;
        }
        if (remaining > 20000 || adjusted || !t_adj)
            return;

        size_t count = std::min<int>(240, std::abs(int(t_adj)));

        if (t_adj > 0) {
            count = std::min<size_t>(count, ma_pcm_rb_available_write(&speaker));
            if (count == 0) return;
            adjusted = true;
            while (count > 0) {
                ma_uint32 frames = static_cast<ma_uint32>(count);
                void* memory = nullptr;
                ma_pcm_rb_acquire_write(&speaker, &frames, &memory);

                std::fill_n(static_cast<float*>(memory), frames, 0.0f);

                ma_pcm_rb_commit_write(&speaker, frames);
                played += frames;
                tx_stamp.sample += frames;
                if (stop_scheduled) stop_sample += frames;
                t_adj -= frames;
                count -= frames;
            }
        } else {
            count = std::min<uint64_t>(count, tx_stamp.sample - played);
            for (auto* channel : tx)
                count = std::min(count, channel->available());

            if (count == 0) return;
            adjusted = true;

            for (auto* channel : tx)
                channel->consume(count);

            tx_stamp.sample -= count;
            if (stop_scheduled) stop_sample -= count;
            t_adj += count;
        }
    }
    u8 update_mask(u8 _mask){
        if (_mask == 0)
            throw std::invalid_argument("Slot mask must contain an active slot");
        slt_mask = _mask;
        while (!((slt_mask >> (7 - tx_stamp.slot_number)) & 1u)) {
            increment_stamp();
        }
        m = mode::armed;
        return tx_stamp.slot_number;
    }
    ma_pcm_rb mic{};
    ma_pcm_rb speaker{};


private:
    u8 slt_mask;
    ma_device device{};
    frame_timing tx_stamp;
    uint64_t played = 0;
    int16_t t_adj = 0;
    bool adjusted = false;
    bool stop_scheduled = false;
    uint64_t stop_sample = 0;
    enum class mode : u8 { wait, armed, play };
    mode m = mode::wait;
    void increment_stamp(){
        tx_stamp.sample += 24000;
        tx_stamp.slot_number +=1;
        if(tx_stamp.slot_number == 8) tx_stamp.slot_number = 0;
    }
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
