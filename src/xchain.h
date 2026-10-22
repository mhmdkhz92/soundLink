#ifndef SOUNDLINK_XCHAIN_H
#define SOUNDLINK_XCHAIN_H

#include "framework.h"
#include "generic.h"
#include "bch.h"
#include "perm.h"
#include "puncture.h"
#include "viterbi.h"
#include "math.h"
#include "filter.h"
#include "physical_layer.h"
#include "dsp.h"
#include "device.h"
#include "data_link.h"

namespace soundlink {

struct tx_channel {
    virtual size_t available() = 0;
    virtual const float* samples() = 0;
    virtual void consume(size_t count) = 0;
    virtual void step() = 0;

    virtual ~tx_channel() = default;
};
struct rx_channel {
    virtual size_t writable() = 0;
    virtual float* samples() = 0;
    virtual void written(size_t count) = 0;
    virtual void step() = 0;

    virtual ~rx_channel() = default;
};

struct txdata_ch:tx_channel{
private:
    scheduler sch;
    const link_cfg cfg;

public:
    txdata_ch(const link_cfg& c, TxMessage& message)
        : sch(), cfg(c),
          data(&sch, "data", 8096),
          coded(&sch, "coded", 8096),
          mixed(&sch, "mixed", 8096),
          bits(&sch, "bits", 8096),
          grid(&sch, "grid", 8096),
          wave(&sch, "wave", 8096),
          audio(&sch, "audio", 25000),
          output(audio),
          source(&sch, message, data),
          bch(&sch, data, coded),
          shuffle(&sch, coded, mixed),
          conv(&sch, mixed, bits, cfg.cr),
          mapper(&sch, bits, grid, cfg),
          ofdm(&sch, grid, wave, cfg),
          up(&sch, wave, audio, cfg) {}

    size_t available() override {
        return output.readable();
    }
    const float* samples() override {
        return output.rd();
    }
    void consume(size_t count) override {
        output.read(count);
    }
    void step() override { sch.step(); }

    txdata_ch(const txdata_ch&) = delete;
    txdata_ch& operator=(const txdata_ch&) = delete;

private:
    pipebuf<u8> data;
    pipebuf<u8> coded;
    pipebuf<u8> mixed;
    pipebuf<u8> bits;
    pipebuf_c<float> grid;
    pipebuf_c<float> wave;
    pipebuf<float> audio;
    pipereader<float> output;

    mtx_loader source;
    bch_encoder<bch127_106> bch;
    interleaver<127, 24> shuffle;
    conv_encoder conv;
    slot_mapper mapper;
    ofdm_modulator ofdm;
    duc up;
};


struct txctrl_ch:tx_channel {
private:
    scheduler sch;
    const link_cfg cfg;

public:
    txctrl_ch(const link_cfg& c, TxctrlQueue& controls)
        : sch(), cfg(c),
          data(&sch, "data", 8096),
          coded(&sch, "coded", 8096),
          mixed(&sch, "mixed", 8096),
          bits(&sch, "bits", 8096),
          grid(&sch, "grid", 8096),
          wave(&sch, "wave", 8096),
          audio(&sch, "audio", 25000),
          output(audio),
          source(&sch, controls, data),
          bch(&sch, data, coded),
          shuffle(&sch, coded, mixed),
          conv(&sch, mixed, bits, cfg.cr),
          mapper(&sch, bits, grid, cfg),
          ofdm(&sch, grid, wave, cfg),
          up(&sch, wave, audio, cfg) {}

    size_t available() override {
        return output.readable();
    }
    const float* samples() override {
        return output.rd();
    }
    void consume(size_t count) override {
        output.read(count);
    }
    void step() override { sch.step(); }

    txctrl_ch(const txctrl_ch&) = delete;
    txctrl_ch& operator=(const txctrl_ch&) = delete;

private:
    pipebuf<u8> data;
    pipebuf<u8> coded;
    pipebuf<u8> mixed;
    pipebuf<u8> bits;
    pipebuf_c<float> grid;
    pipebuf_c<float> wave;

    pipebuf<float> audio;
    pipereader<float> output;

    ctrl_loader source;
    bch_encoder<bch63_36> bch;
    interleaver<63, 8> shuffle;
    conv_encoder conv;
    slot_mapper mapper;
    ofdm_modulator ofdm;
    duc up;
};

struct rxdata_ch:rx_channel {
private:
    scheduler sch;
    const link_cfg cfg;
    pipebuf<float> audio;
    pipewriter<float> input;

public:

    rxdata_ch(const link_cfg& c, RxMessage& message, TxctrlQueue& controls)
        : sch(), cfg(c),
          audio(&sch, "audio", 48000),
          input(audio),
          wave(&sch, "wave", 8096),
          grid(&sch, "grid", 8096),
          bits(&sch, "bits", 24000),
          mixed(&sch, "mixed", 8096),
          coded(&sch, "coded", 8096),
          data(&sch, "data", 8096),
          down(&sch, audio, wave, cfg),
          ofdm(&sch, wave, grid, cfg),
          mapper(&sch, grid, bits, cfg),
          conv(&sch, bits, mixed, cfg.cr),
          shuffle(&sch, mixed, coded),
          bch(&sch, coded, data),
          sink(&sch, data, message, controls) {}

    size_t writable() override {
        return input.writable();
    }
    float* samples() override {
        return input.wr();
    }
    void written(size_t count) override {
        input.written(count);
    }
    void step() override { sch.step(); }

    rxdata_ch(const rxdata_ch&) = delete;
    rxdata_ch& operator=(const rxdata_ch&) = delete;

private:
    pipebuf_c<float> wave;
    pipebuf_c<float> grid;
    pipebuf<u8> bits;
    pipebuf<u8> mixed;
    pipebuf<u8> coded;
    pipebuf<u8> data;

    ddc down;
    ofdm_demodulator ofdm;
    slot_demapper mapper;
    viterbi conv;
    deinterleaver<127, 24> shuffle;
    bch_decoder<bch127_106> bch;
    mrx_store sink;
};


struct rxctrl_ch:rx_channel {
private:
    scheduler sch;
    const link_cfg cfg;
    pipebuf<float> audio;
    pipewriter<float> input;

public:
    rxctrl_ch(const link_cfg& c, rxCtrlQueue& controls)
        : sch(), cfg(c),
          audio(&sch, "audio", 48000),
          input(audio),
          wave(&sch, "wave", 8096),
          grid(&sch, "grid", 8096),
          bits(&sch, "bits", 24000),
          mixed(&sch, "mixed", 8096),
          coded(&sch, "coded", 8096),
          data(&sch, "data", 8096),
          down(&sch, audio, wave, cfg),
          ofdm(&sch, wave, grid, cfg),
          mapper(&sch, grid, bits, cfg),
          conv(&sch, bits, mixed, cfg.cr),
          shuffle(&sch, mixed, coded),
          bch(&sch, coded, data),
          sink(&sch, data, controls) {}

    size_t writable() override {
        return input.writable();
    }
    float* samples() override {
        return input.wr();
    }
    void written(size_t count) override {
        input.written(count);
    }
    void step() override { sch.step(); }

    rxctrl_ch(const rxctrl_ch&) = delete;
    rxctrl_ch& operator=(const rxctrl_ch&) = delete;

private:
    pipebuf_c<float> wave;
    pipebuf_c<float> grid;
    pipebuf<u8> bits;
    pipebuf<u8> mixed;
    pipebuf<u8> coded;
    pipebuf<u8> data;

    ddc down;
    ofdm_demodulator ofdm;
    slot_demapper mapper;
    viterbi conv;
    deinterleaver<63, 8> shuffle;
    bch_decoder<bch63_36> bch;
    ctrl_store sink;
};

inline void mix_audio(const std::vector<tx_channel*>& tx, ma_pcm_rb& playback) {
    if (tx.empty()) return;
    size_t count = ma_pcm_rb_available_write(&playback);

    for (auto* channel : tx)
        count = std::min(count, channel->available());

    const float gain = 1.0f / float(tx.size());

    while (count > 0) {
        ma_uint32 frames = static_cast<ma_uint32>(count);
        void* memory = nullptr;
        ma_pcm_rb_acquire_write(&playback, &frames, &memory);

        float* output = static_cast<float*>(memory);
        std::fill_n(output, frames, 0.0f);

        for (auto* channel : tx) {
            ma_mix_pcm_frames_f32(output, channel->samples(), frames, 1, gain);
            channel->consume(frames);
        }
        ma_pcm_rb_commit_write(&playback, frames);
        count -= frames;
    }
}

inline void read_mic(const std::vector<rx_channel*>& rx, ma_pcm_rb& capture) {
    if (rx.empty()) return;
    size_t count = ma_pcm_rb_available_read(&capture);

    for (auto* channel : rx)
        count = std::min(count, channel->writable());

    while (count > 0) {
        ma_uint32 frames = static_cast<ma_uint32>(count);
        void* memory = nullptr;
        ma_pcm_rb_acquire_read(&capture, &frames, &memory);

        for (auto* channel : rx) {
            std::memcpy(channel->samples(), memory, frames * sizeof(float));
            channel->written(frames);
        }
        ma_pcm_rb_commit_read(&capture, frames);
        count -= frames;
    }
}

} // namespace soundlink

#endif
