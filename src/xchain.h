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


struct txdata_ch:tx_channel{
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
    scheduler sch;
    const link_cfg cfg;
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
    scheduler sch;
    const link_cfg cfg;
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


struct rx_stamp {
    frame_timing ft;
    bool valid = false;
    rx_stamp(const link_cfg& cfg)
        : ratio(48000.0 / cfg.sr), offset(0) {
        // DDC sample alignment derived from the current filter implementation.
        switch (cfg.bw) {
        case BW::BW1_4: offset = 168.0; break;
        case BW::BW3:   offset = 103.5; break;
        case BW::BW5:   offset = 32.5; break;
        case BW::BW10:  offset = 15.75; break;
        }
    }
    void update(const pss_tracker& pt, const meta_timing& mt) {
        if (mt.processed_grids == checked_grid)
            return;
        checked_grid = mt.processed_grids;
        if (!pt.pss_valid || !mt.successful_last)
            return;
        if (mt.processed_grids < pt.pss_grid)
            return;
        u32 elapsed = mt.processed_grids - pt.pss_grid;
        ft.slot_number = (mt.sltn + 8 - (elapsed & 7u)) & 7u;

        ft.sample = static_cast<uint64_t>(
            std::llround(double(pt.pss_sample) * ratio + offset));
        valid = true;
    }

private:
    double ratio;
    double offset;
    u32 checked_grid = 0;
};
struct rxdata_ch:rx_channel {

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
          sink(&sch, data, message, controls),
          stamp(cfg) {}

    size_t writable() override {
        return input.writable();
    }
    float* samples() override {
        return input.wr();
    }
    void written(size_t count) override {
        input.written(count);
    }
    void step() override {
        sch.step();
        stamp.update(ofdm.pt, mapper.mt);
    }
    bool get_stamp(frame_timing& ft) override {
        if (!stamp.valid) return false;
        ft = stamp.ft;
        return true;
    }

    rxdata_ch(const rxdata_ch&) = delete;
    rxdata_ch& operator=(const rxdata_ch&) = delete;

private:
    scheduler sch;
    const link_cfg cfg;
    pipebuf<float> audio;
    pipewriter<float> input;
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
    rx_stamp stamp;
};


struct rxctrl_ch:rx_channel {
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
          sink(&sch, data, controls),
          stamp(cfg) {}

    size_t writable() override {
        return input.writable();
    }
    float* samples() override {
        return input.wr();
    }
    void written(size_t count) override {
        input.written(count);
    }
    void step() override {
        sch.step();
        stamp.update(ofdm.pt, mapper.mt);
    }
    bool get_stamp(frame_timing& ft) override {
        if (!stamp.valid) return false;
        ft = stamp.ft;
        return true;
    }

    rxctrl_ch(const rxctrl_ch&) = delete;
    rxctrl_ch& operator=(const rxctrl_ch&) = delete;

private:
    scheduler sch;
    const link_cfg cfg;
    pipebuf<float> audio;
    pipewriter<float> input;
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
    rx_stamp stamp;
};



} // namespace soundlink

#endif
