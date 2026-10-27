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
    txdata_ch(const link_cfg& _cfg, TxMessage& message, u8 mask)
        : sch(), cfg(_cfg),
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
          mapper(&sch, bits, grid, cfg, mask),
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

    bool run() override {
        uint64_t before = audio.total_written;
        for (int i = 0; i < 10; ++i) {
            sch.step();
            if (audio.total_written != before) return true;
        }
        return false;
    }

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
    txctrl_ch(const link_cfg& _cfg, TxctrlQueue& controls, u8 mask)
        : sch(), cfg(_cfg),
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
          mapper(&sch, bits, grid, cfg, mask),
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

    bool run() override {
        uint64_t before = audio.total_written;
        for (int i = 0; i < 10; ++i) {
            sch.step();
            if (audio.total_written != before) return true;
        }
        return false;
    }

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

    rxdata_ch(const link_cfg& _cfg, RxMessage& message, TxctrlQueue& controls, u8 mask)
        : sch(), cfg(_cfg),
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
          mapper(&sch, grid, bits, cfg, mask),
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
    rxctrl_ch(const link_cfg& _cfg, rxCtrlQueue& controls, u8 mask)
        : sch(), cfg(_cfg),
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
          mapper(&sch, grid, bits, cfg, mask),
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


// STATE MACHINES
inline constexpr u8 h_TDD = 0b11001100;  // TDD pattern in handshake
inline constexpr u8 t_TDD = 0b11101110;  // TDD pattern in transfer

const link_cfg h_cfg[] = {
    {4, BW::BW1_4, FEC12},
    {8, BW::BW1_4, FEC12},
    {12, BW::BW1_4, FEC12},
    {16, BW::BW1_4, FEC12}
};
struct sender {
    enum class state { handshake, transfer, close };
    state current = state::handshake;
    sender(const std::filesystem::path& path)
        : ad(h_TDD),
          msg(path, 160),
          txctrl(20),
          rxctrl(20) {
        for (int i = 0; i < 4; ++i) {
            tx.push_back(new txctrl_ch(h_cfg[i], txctrl, h_TDD));
            rx.push_back(new rxctrl_ch(h_cfg[i], rxctrl, u8(~h_TDD)));
        }
        txctrl.push(ctrl_factory::hello(msg.metadata));
        ad.play();
        while (ma_pcm_rb_available_read(&ad.speaker) < TARGET_FILL) {
            for (auto* ch : tx) ch->run();
            ad.mix_audio(tx);
        }
        ad.start();
    }
    void step() {
        switch (current) {
        case state::handshake: {
            ad.read_mic(rx);
            for (auto* ch : rx) ch->step();
            for (auto* ch : tx) ch->run();
            ad.mix_audio(tx);

            while (!rxctrl.empty()) {
                const ctrl_packet& packet = rxctrl.next();

                switch (packet.type) {
                case ctrl_type::hello_ack: {
                    uint16_t sequence = (uint16_t(packet.fields[0]) << 8) | packet.fields[1];
                    if (sequence == hello_sequence)
                        hello_acked = true;
                    // Keep hello transmitting until its replacement is queued.
                    break;
                }
                case ctrl_type::config: {
                    read_config(packet, data_cfg, ctrl_cfg);
                    if (!txctrl.push(ctrl_factory::config_ack(packet.sequence)))
                        return;  // Retain this RX packet and retry next step.

                    txctrl.remove(hello_sequence);
                    config_ready = true;
                    break;
                }
                default:
                    break;
                }
                rxctrl.pop();
            }

            // Stay in handshake until the agreed silent switchover.
            break;
        }
        case state::transfer:
            // transfer code

            break;
        case state::close:

            break;
        }
    }

    ~sender() {
        for (auto* ch : tx) delete ch;
        for (auto* ch : rx) delete ch;
    }

private:
    audio_device ad;
    TxMessage msg;
    TxctrlQueue txctrl;
    rxCtrlQueue rxctrl;
    std::vector<tx_channel*> tx;
    std::vector<rx_channel*> rx;

    // state flags and notes:
    bool hello_acked = false;
    bool config_ready = false;
    uint16_t hello_sequence = 0;
    chinfo data_cfg;
    chinfo ctrl_cfg;
};

} // namespace soundlink

#endif
