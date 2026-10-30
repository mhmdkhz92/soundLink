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

    void set_sltn(u8 n) override { mapper.set_sltn(n); }

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

    void set_sltn(u8 n) override { mapper.set_sltn(n); }

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
        case BW1_4: offset = 168.0; break;
        case BW3:   offset = 103.5; break;
        case BW5:   offset = 32.5; break;
        case BW10:  offset = 15.75; break;
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

    void set_mask(u8 mask) override { mapper.set_mask(mask); }

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
        stamp.valid = false;
        return true;
    }
    void clear_stamp() override {
        stamp.valid = false;
        ofdm.pt.pss_valid = false;
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

    void set_mask(u8 mask) override { mapper.set_mask(mask); }

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
        stamp.valid = false;
        return true;
    }
    void clear_stamp() override {
        stamp.valid = false;
        ofdm.pt.pss_valid = false;
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


inline bool calculate_time(const std::vector<rx_channel*>& rx, int16_t& adjustment) {
    int64_t total = 0;
    int count = 0;

    for (auto* ch : rx) {
        frame_timing stamp;
        if (!ch->get_stamp(stamp)) continue;

        int error = int(stamp.slot_number) * 24000 - int(stamp.sample % 192000);
        if (error >= 96000) error -= 192000;
        if (error < -96000) error += 192000;
        total += error;
        ++count;
    }
    if (count == 0) return false;
    int64_t correction = total / count;
    if (correction > 1000) correction = 1000;
    if (correction < -1000) correction = -1000;
    adjustment = (int16_t)correction;
    return true;
}


// STATE MACHINES
inline constexpr u8 h_TDD = 0b11001100;  // TDD pattern in handshake
inline constexpr u8 t_TDD = 0b11101110;  // TDD pattern in transfer

const link_cfg h_cfg[] = {
    {4, BW1_4, FEC12},
    {8, BW1_4, FEC12},
    {12,BW1_4, FEC12},
    {16,BW1_4, FEC12}
};

// Fixed transfer configuration; center frequencies are in kHz.
inline const chinfo fixed_data_cfg = {
    {3, FEC34, BW3, QPSK},
    {7, FEC34, BW5, QPSK},
    {11,FEC34, BW3, QAM16}
};
inline const chinfo fixed_ctrl_cfg = {
    {15, FEC12, BW3, QPSK}
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
        if (finished) return;
        if(acked_timing && calculate_time(rx, timing_report)){
            if (txctrl.push(ctrl_factory::timing_adjustment(timing_report)))
                acked_timing = false;
        }
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
                    if (packet.fields.size() != 2) break;
                    uint16_t sequence = (uint16_t(packet.fields[0]) << 8) | packet.fields[1];
                    if (sequence == hello_sequence)
                        acked_hello = true;
                    // Keep hello transmitting until its replacement is queued.
                    break;
                }
                case ctrl_type::config: {
                    read_config(packet, data_cfg, ctrl_cfg);
                    txctrl.remove(hello_sequence);
                    txctrl.push(ctrl_factory::config_ack(packet.sequence));
                    // ready for tansission mode
                    u8 next_active_slot = ad.update_mask(t_TDD);
                    create_tx();
                    for(auto* ch: tx){
                        ch->set_sltn(next_active_slot);
                        ch->run();
                    }
                    for (auto* ch : rx)
                        ch->set_mask(u8(~t_TDD));
                    acked_cfg = true;
                    current = state::transfer;
                    
                    break;
                }
                case ctrl_type::timing_ack: {
                    if (packet.fields.size() != 2) break;
                    uint16_t sequence = (uint16_t(packet.fields[0]) << 8) | packet.fields[1];
                    txctrl.remove(sequence);
                    acked_timing = true;
                    for (auto* ch : rx)
                        ch->clear_stamp();
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
        case state::transfer:{
            if(txctrl.size() > 1) txctrl.remove_type(ctrl_type::config_ack);
            ad.read_mic(rx);
            for (auto* ch : rx) ch->step();

            while (!rxctrl.empty()) {
                const ctrl_packet& packet = rxctrl.next();
                switch (packet.type) {
                case ctrl_type::ack: {
                    if (packet.fields.size() != 6) break;
                    for (uint16_t segment : read_ack(packet))
                        msg.acknowledge(segment);
                    break;
                }
                case ctrl_type::timing_ack: {
                    if (packet.fields.size() != 2) break;
                    uint16_t sequence = (uint16_t(packet.fields[0]) << 8) | packet.fields[1];
                    txctrl.remove(sequence);
                    acked_timing = true;
                    for (auto* ch : rx) ch->clear_stamp();
                    break;
                }
                default:
                    break;
                }
                rxctrl.pop();
            }
            if (msg.completed()) {
                for (size_t i = 0; i < data_cfg.size(); ++i)
                    delete tx[i];

                tx.erase(tx.begin(), tx.begin() + data_cfg.size());

                txctrl.push(ctrl_factory::close());
                current = state::close;
            }

            for (auto* ch : tx) ch->run();
            ad.mix_audio(tx);
        }
            
            break;
        case state::close: {
            ad.read_mic(rx);
            for (auto* ch : rx) ch->step();

            while (!rxctrl.empty()) {
                const ctrl_packet& packet = rxctrl.next();

                switch (packet.type) {
                case ctrl_type::close_ack: {
                    if (packet.fields.size() != 2) break;
                    ad.stop();
                    finished = true;
                    break;
                }
                case ctrl_type::timing_ack: {
                    if (packet.fields.size() != 2) break;
                    uint16_t sequence = (uint16_t(packet.fields[0]) << 8) | packet.fields[1];
                    txctrl.remove(sequence);
                    acked_timing = true;
                    for (auto* ch : rx) ch->clear_stamp();
                    break;
                }
                default:
                    break;
                }

                rxctrl.pop();
                if (finished) return;
            }

            for (auto* ch : tx) ch->run();
            ad.mix_audio(tx);
            break;
        }
        }
    }
    ~sender() {
        for (auto* ch : tx) delete ch;
        for (auto* ch : rx) delete ch;
    }
    void create_tx() {
        for (auto* ch : tx) delete ch;
        tx.clear();
        for (const auto& ch : data_cfg) {
            link_cfg cfg(ch.fc, BW(ch.bandwidth), code_rate(ch.code_rate), modulation(ch.mod));
            tx.push_back(new txdata_ch(cfg, msg, t_TDD));
        }
        for (const auto& ch : ctrl_cfg) {
            link_cfg cfg(ch.fc, BW(ch.bandwidth), code_rate(ch.code_rate), modulation(ch.mod));
            tx.push_back(new txctrl_ch(cfg, txctrl, t_TDD));
        }
    }

private:
    audio_device ad;
    TxMessage msg;
    TxctrlQueue txctrl;
    rxCtrlQueue rxctrl;
    std::vector<tx_channel*> tx;
    std::vector<rx_channel*> rx;

    // state flags and notes:
    bool acked_hello = false;
    bool acked_cfg = false;
    bool acked_timing = true;
    bool finished = false;
    int16_t timing_report;
    uint16_t hello_sequence = 0;
    chinfo data_cfg;
    chinfo ctrl_cfg;
};

inline bool average_timing(const std::vector<rx_channel*>& rx, frame_timing& result){
    std::vector<frame_timing> measurements;
    frame_timing reference;

    for (rx_channel* ch : rx) {
        frame_timing stamp;
        if (!ch->get_stamp(stamp)) continue;

        if (measurements.empty() || stamp.sample > reference.sample)
            reference = stamp;

        measurements.push_back(stamp);
    }

    if (measurements.empty()) return false;

    int64_t total_offset = 0;
    for (const frame_timing& stamp : measurements) {
        int64_t offset =(int64_t)stamp.sample - (int64_t)reference.sample;

        offset += (int(reference.slot_number) - int(stamp.slot_number)) * 24000;

        // Project onto the nearest occurrence of the reference slot.
        offset %= 192000;
        if (offset >= 96000) offset -= 192000;
        if (offset < -96000) offset += 192000;

        total_offset += offset;
    }

    int64_t average_offset = total_offset / (int64_t)measurements.size();

    int64_t sample = (int64_t)reference.sample + average_offset;
    if (sample < 0) return false;

    result.sample = (uint64_t)sample;
    result.slot_number = reference.slot_number;
    return true;
}
struct receiver {
    enum class state { handshake, transfer, close };
    state current = state::handshake;

    receiver(const std::filesystem::path& path)
        : ad(u8(~h_TDD)),
          output_path(path),
          txctrl(20),
          rxctrl(20) {
        for (const auto& cfg : h_cfg) {
            rx.push_back(new rxctrl_ch(cfg, rxctrl, h_TDD));
        }

        while (ma_pcm_rb_available_read(&ad.speaker) < TARGET_FILL)
            ad.mix_audio(tx);
        ad.start();
    }

    void step() {
        if (finished) return;

        switch (current) {
        case state::handshake: {
            ad.read_mic(rx);
            for (auto* ch : rx) ch->step();

            while (!rxctrl.empty()) {
                const ctrl_packet& packet = rxctrl.next();

                switch (packet.type) {
                case ctrl_type::hello: {
                    if (msg) break;

                    file_info file = read_hello(packet);
                    msg = new RxMessage(output_path.append(file.file_name),
                        file.file_size, file.segment_size);
                    txctrl.push(ctrl_factory::hello_ack(packet.sequence));
                    break;
                }
                case ctrl_type::timing_adj: {
                    handle_timing(packet);
                    if (timing_sequence)
                        pre_defined_conf = true;
                    break;
                }
                default:
                    break;
                }

                rxctrl.pop();
            }

            if (!tx_init && !txctrl.empty()) {
                frame_timing timing;
                if (average_timing(rx, timing)) {
                    u8 next_slot = ad.next_opportunity(&timing);
                    create_tx(u8(~h_TDD), next_slot);
                    ad.arm();
                    tx_init = true;
                }
            }

            for (tx_channel* ch : tx) ch->run();
            ad.mix_audio(tx);

            if (pre_defined_conf && tx_init && msg) {
                txctrl.push(ctrl_factory::config(fixed_data_cfg, fixed_ctrl_cfg));
                create_rx(fixed_data_cfg, fixed_ctrl_cfg);

                u8 next_slot = ad.update_mask(u8(~t_TDD));
                create_tx(u8(~t_TDD), next_slot);

                current = state::transfer;
            }

            break;
        }
        case state::transfer: {
            ad.read_mic(rx);
            for (rx_channel* ch : rx) ch->step();

            if (!transfer_confirmed &&
                (msg->received_count > 0 || !rxctrl.empty())) {
                txctrl.remove_type(ctrl_type::hello_ack);
                txctrl.remove_type(ctrl_type::config);
                transfer_confirmed = true;
            }

            while (!rxctrl.empty()) {
                const ctrl_packet& packet = rxctrl.next();

                switch (packet.type) {
                case ctrl_type::timing_adj: {
                    handle_timing(packet);
                    break;
                }
                case ctrl_type::close: {
                    if (!msg->complete()) break;

                    msg->flush_to_file();
                    txctrl.remove_type(ctrl_type::ack);
                    txctrl.push(ctrl_factory::close_ack(packet.sequence));
                    ad.stop_after_active_slots(4);
                    current = state::close;
                    break;
                }
                default:
                    break;
                }

                rxctrl.pop();
                if (current == state::close) break;
            }

            for (tx_channel* ch : tx) ch->run();
            ad.mix_audio(tx);
            break;
        }
        case state::close: {
            ad.read_mic(rx);
            for (rx_channel* ch : rx) ch->step();

            while (!rxctrl.empty()) {
                const ctrl_packet& packet = rxctrl.next();

                switch (packet.type) {
                case ctrl_type::timing_adj: {
                    handle_timing(packet);
                    break;
                }
                default:
                    break;
                }

                rxctrl.pop();
            }

            for (tx_channel* ch : tx) ch->run();
            ad.mix_audio(tx);
            if (ad.stop_ready()) {
                ad.stop();
                finished = true;
                return;
            }
            break;
        }
        }

        if (timing_sequence && ad.adjustment_complete()) {
            if (txctrl.push(ctrl_factory::timing_ack(timing_sequence)))
                timing_sequence = 0;
        }
    }

    ~receiver() {
        for (auto* ch : tx) delete ch;
        for (auto* ch : rx) delete ch;
        delete msg;
    }

    receiver(const receiver&) = delete;
    receiver& operator=(const receiver&) = delete;

private:
    void handle_timing(const ctrl_packet& packet) {
        if (timing_sequence || !ad.adjustment_complete() ||
            packet.fields.size() != 2)
            return;

        int16_t adjustment = (int16_t)(
            (uint16_t(packet.fields[0]) << 8) | packet.fields[1]);
        if (adjustment < -1000 || adjustment > 1000) adjustment = 0;

        timing_sequence = packet.sequence;
        ad.set_adjustment(adjustment);
    }

    void create_tx(u8 mask, u8 slot_number) {
        for (tx_channel* ch : tx) delete ch;
        tx.clear();

        for (const link_cfg& cfg : h_cfg) {
            txctrl_ch* ch = new txctrl_ch(cfg, txctrl, mask);
            ch->set_sltn(slot_number);
            tx.push_back(ch);
        }
        for (tx_channel* ch : tx) {
            while (ch->available() < TARGET_FILL) {
                if (!ch->run())
                    throw std::runtime_error("Cannot prefill receiver TX channel");
            }
        }
    }

    void create_rx(chinfo data_cfg, chinfo ctrl_cfg) {
        for (rx_channel* ch : rx) delete ch;
        rx.clear();

        for (const channel_info& ch : data_cfg) {
            link_cfg cfg(ch.fc, BW(ch.bandwidth), code_rate(ch.code_rate), modulation(ch.mod));
            rx.push_back(new rxdata_ch(cfg, *msg, txctrl, t_TDD));
        }

        for (const channel_info& ch : ctrl_cfg) {
            link_cfg cfg(ch.fc, BW(ch.bandwidth), code_rate(ch.code_rate), modulation(ch.mod));
            rx.push_back(new rxctrl_ch(cfg, rxctrl, t_TDD));
        }
    }

    audio_device ad;
    std::filesystem::path output_path;
    RxMessage* msg = nullptr;

    TxctrlQueue txctrl;
    rxCtrlQueue rxctrl;

    std::vector<tx_channel*> tx;
    std::vector<rx_channel*> rx;

    // flow control
    uint16_t timing_sequence = 0;
    bool tx_init = false;
    bool pre_defined_conf = false;
    bool transfer_confirmed = false;
    bool finished = false;
};

} // namespace soundlink

#endif
