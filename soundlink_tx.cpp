#define MINIAUDIO_IMPLEMENTATION
#include "src/miniaudio.h"

#include "src/framework.h"
#include "src/generic.h"
#include "src/bch.h"
#include "src/perm.h"
#include "src/viterbi.h"
#include "src/math.h"
#include "src/physical_layer.h"
#include "src/dsp.h"
#include "src/device.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <thread>
#include <atomic>
#include <chrono>



void speaker_callback(ma_device* device, void* output, const void*, ma_uint32 frame_count){
    auto& ring = *static_cast<ma_pcm_rb*>(device->pUserData);
    auto* samples = static_cast<float*>(output);

    ma_uint32 copied = 0;

    while (copied < frame_count) {
        ma_uint32 count = frame_count - copied;
        void* memory = nullptr;

        if (ma_pcm_rb_acquire_read(&ring, &count, &memory) != MA_SUCCESS || count == 0) {
            break;
        }

        std::memcpy(samples + copied, memory, count * sizeof(float));
        ma_pcm_rb_commit_read(&ring, count);
        copied += count;
    }
        // Always fill the entire output buffer.
    if (copied < frame_count) {
        const ma_uint32 missing = frame_count - copied;
        std::memset(samples + copied, 0, missing * sizeof(float));
        fprintf(stdout, "U");
    }
}

using namespace soundlink;
int main(int argc, char* argv[])
{
    code_rate cr = FEC12;
    BW bw = BW::BW1_4;
    int fc = 5;
    bool have_cr = false;
    bool have_bw = false;

    for (int i = 1; i < argc; ++i) {
        const char* option = argv[i];

        if (std::strcmp(option, "--cr") == 0) {

            const char* value = argv[++i];
            if (std::strcmp(value, "1/2") == 0) cr = FEC12;
            else if (std::strcmp(value, "2/3") == 0) cr = FEC23;
            else if (std::strcmp(value, "3/4") == 0) cr = FEC34;
            else if (std::strcmp(value, "5/6") == 0) cr = FEC56;
            else if (std::strcmp(value, "7/8") == 0) cr = FEC78;
            else {
                std::fprintf(stderr, "--cr must be 1/2, 2/3, 3/4, 5/6, or 7/8.\n");
                return 1;
            }
            have_cr = true;
        } else if (std::strcmp(option, "--bw") == 0) {

            const char* value = argv[++i];
            if (std::strcmp(value, "1.4") == 0) bw = BW::BW1_4;
            else if (std::strcmp(value, "3") == 0) bw = BW::BW3;
            else if (std::strcmp(value, "5") == 0) bw = BW::BW5;
            else if (std::strcmp(value, "10") == 0) bw = BW::BW10;
            else {
                std::fprintf(stderr, "--bw must be 1.4, 3, 5, or 10 (kHz).\n");
                return 1;
            }
            have_bw = true;
        } else if (std::strcmp(option, "--fc") == 0) {
            const char* value = argv[++i];
            char* end = nullptr;
            const long parsed = std::strtol(value, &end, 10);
            if (end == value || *end != '\0' || parsed < 2 || parsed > 18) {
                std::fprintf(stderr, "--fc must be an integer from 2 to 18 (kHz).\n");
                return 1;
            }
            fc = static_cast<int>(parsed);
        } else {
            std::fprintf(stderr, "Unknown argument: %s\n", option);
            return 1;
        }
    }
    if (!have_cr || !have_bw) {
        std::fprintf(stderr, "Usage: %s --cr RATE --bw BANDWIDTH_KHZ [--fc CENTER_KHZ]\n", argv[0]);
        return 1;
    }

    
    // audio components
    ma_pcm_rb ring;
    std::atomic<bool> stop{false};
    if (ma_pcm_rb_init(ma_format_f32, 1, CAPACITY, nullptr, nullptr, &ring) != MA_SUCCESS) {
        std::fprintf(stderr, "Could not initialize ring.\n");
        return 1;
    }
    ma_device_config config = ma_device_config_init(ma_device_type_playback);

    config.playback.format   = ma_format_f32;
    config.playback.channels = 1;
    config.sampleRate        = SAMPLE_RATE;
    config.dataCallback      = speaker_callback;
    config.pUserData         = &ring;

    ma_device device;

    if (ma_device_init(nullptr, &config, &device) != MA_SUCCESS) {
        std::fprintf(stderr, "Could not open speaker.\n");
        ma_pcm_rb_uninit(&ring);
        return 1;
    }

    scheduler tx;
    link_cfg cfg(bw);
    // buffers
    pipebuf<u8> pipe_data(&tx, "input pipe", 8096);
    pipebuf<u8> pipe_bch(&tx, "bch pipe", 8096);
    pipebuf<u8> pipe_interleaver(&tx, "interleaved pipe", 8096);
    pipebuf<u8> pipe_convcode(&tx, "convo enc pipe", 8096);
    pipebuf_c<float> pipe_slot(&tx, "slot pipe", 3*8096);
    pipebuf_c<float> pipe_time(&tx, "ifft pipe", 3*8096);
    pipebuf<float> pipe_time48(&tx, "48kHz pipe", 3*8096);

    // runnables
    prbs_generator prbs(&tx, pipe_data);
    bch_encoder<bch127_106> bch_c(&tx, pipe_data, pipe_bch);
    interleaver<127, 24> interle(&tx, pipe_bch, pipe_interleaver);
    conv_encoder cnv_enc(&tx, pipe_interleaver, pipe_convcode, cr);
    slot_mapper sltm(&tx, pipe_convcode, pipe_slot, cfg);
    ofdm_modulator mod(&tx, pipe_slot, pipe_time, cfg);
    duc rate(&tx, pipe_time, pipe_time48, cfg);
    rate.set_fc(fc);
    speaker_sink sink(&tx, pipe_time48, ring);


    while (ma_pcm_rb_available_read(&ring) < TARGET_FILL) {
        const uint64_t before = tx.hash();
        tx.step();

        if (tx.hash() == before) {
            std::fprintf(stderr, "TX stalled during prefill.\n");
            ma_device_uninit(&device);
            ma_pcm_rb_uninit(&ring);
            return 1;
        }
    }
    std::thread worker([&] {
        while (!stop.load()) {
            if (ma_pcm_rb_available_read(&ring) >= SLEEP_THRESHOLD) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }

            const uint64_t before = tx.hash();
            tx.step();

            if (tx.hash() == before) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    });

    if (ma_device_start(&device) != MA_SUCCESS) {
        std::fprintf(stderr, "Could not start speaker.\n");

        stop.store(true);
        worker.join();
        ma_device_uninit(&device);
        ma_pcm_rb_uninit(&ring);
        return 1;
    }

    std::puts("Press Enter to stop.");
    std::getchar();

    // Stop the callback while its state and ring still exist.
    ma_device_uninit(&device);

    stop.store(true);
    worker.join();



    ma_pcm_rb_uninit(&ring);
    return 0;
}
