#ifndef SOUNDLINK_DEVICE_H
#define SOUNDLINK_DEVICE_H


#include "framework.h"
#include "miniaudio.h"

constexpr ma_uint32 SAMPLE_RATE = 48000;
constexpr ma_uint32 CAPACITY    = SAMPLE_RATE;      // 1 second.
constexpr ma_uint32 TARGET_FILL = SAMPLE_RATE / 4;  // 250 ms.
constexpr ma_uint32 SLEEP_THRESHOLD = SAMPLE_RATE / 5;

namespace soundlink{

struct speaker_sink : runnable {
    speaker_sink(scheduler* sch, pipebuf<float>& _in, ma_pcm_rb& _ring)
        :runnable(sch, "SPEAKER_SINK"),
        in(_in),
        ring(_ring){}

    void run() override {
        const ma_uint32 queued = ma_pcm_rb_available_read(&ring);

        if (queued >= TARGET_FILL) {
            return;
        }

        ma_uint32 remaining = TARGET_FILL - queued;

        while (remaining > 0) {
            const size_t available = in.readable();

            if (available == 0) {
                break;
            }

            ma_uint32 count = remaining;

            if (available < count) {
                count = static_cast<ma_uint32>(available);
            }

            void* memory = nullptr;

            if (ma_pcm_rb_acquire_write(&ring, &count, &memory) != MA_SUCCESS ||count == 0) {
                break;
            }

            std::memcpy(memory, in.rd(), count * sizeof(float));

            ma_pcm_rb_commit_write(&ring, count);
            in.read(count);

            remaining -= count;
        }
    }

private:
    pipereader<float> in;
    ma_pcm_rb& ring;
};

}

#endif