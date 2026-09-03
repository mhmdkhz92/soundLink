#include <chrono>
#include <cstdio>

#include "dsp.h"
#include "generic.h"

using namespace soundlink;

int main() {
    constexpr unsigned long buffer_size = 16384;

    /*
       Stage 1: 48 kHz -> 8 kHz

       The first frequency that can alias into the final 0..900 Hz
       passband is 8000 - 900 = 7100 Hz. Therefore this stage may
       transition from 900 Hz to 7100 Hz.
    */
    constexpr float stage1_cutoff =
        4000.0f / 48000.0f;  // Midpoint of 900 and 7100 Hz

    constexpr float stage1_bandwidth =
        1.0f - 0.5f * (6200.0f / 48000.0f);

    /*
       Stage 2: 8 kHz -> 4 kHz

       The first frequency that can alias into 0..900 Hz is
       4000 - 900 = 3100 Hz. This stage may transition from
       900 Hz to 3100 Hz.
    */
    constexpr float stage2_cutoff =
        2000.0f / 8000.0f;  // Midpoint of 900 and 3100 Hz

    constexpr float stage2_bandwidth =
        1.0f - 0.5f * (2200.0f / 8000.0f);

    /*
       Stage 3: 4 kHz -> 2 kHz

       Keep the original final response:
       passband edge = 900 Hz, stopband edge = 1000 Hz.
    */
    constexpr float stage3_cutoff =
        950.0f / 4000.0f;  // Midpoint of 900 and 1000 Hz

    constexpr float stage3_bandwidth =
        1.0f - 0.5f * (100.0f / 4000.0f);

    scheduler sch;

    pipebuf<float> input_48k(&sch, "input_48k", buffer_size);
    pipebuf<float> stage_8k(&sch, "stage_8k", buffer_size);
    pipebuf<float> stage_4k(&sch, "stage_4k", buffer_size);
    pipebuf<float> output_2k(&sch, "output_2k", buffer_size);

    file_reader<float> reader(&sch, "input", input_48k);

    resampler<1, 6> downsample_48k_to_8k(
        &sch,
        input_48k,
        stage_8k,
        stage1_cutoff,
        stage1_bandwidth
    );

    resampler<1, 2> downsample_8k_to_4k(
        &sch,
        stage_8k,
        stage_4k,
        stage2_cutoff,
        stage2_bandwidth
    );

    resampler<1, 2> downsample_4k_to_2k(
        &sch,
        stage_4k,
        output_2k,
        stage3_cutoff,
        stage3_bandwidth
    );

    file_writer<float> writer(&sch, output_2k, "output");

    const auto start = std::chrono::steady_clock::now();

    sch.run();
    sch.shutdown();

    const auto finish = std::chrono::steady_clock::now();
    const double elapsed_seconds =
        std::chrono::duration<double>(finish - start).count();

    std::printf("C++ elapsed time: %.6f seconds\n", elapsed_seconds);
    std::printf("48 kHz input samples: %lu\n", input_48k.total_written);
    std::printf("8 kHz stage samples:  %lu\n", stage_8k.total_written);
    std::printf("4 kHz stage samples:  %lu\n", stage_4k.total_written);
    std::printf("2 kHz output samples: %lu\n", output_2k.total_written);
    std::printf(
        "C++ input throughput: %.3f million samples/second\n",
        static_cast<double>(input_48k.total_written) /
            elapsed_seconds / 1.0e6
    );

    return 0;
}
