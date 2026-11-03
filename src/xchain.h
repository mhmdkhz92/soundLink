#ifndef SOUNDLINK_XCHAIN_H
#define SOUNDLINK_XCHAIN_H

#include "framework.h"
#include "bch.h"
#include "perm.h"
#include "viterbi.h"
#include "physical_layer.h"
#include "dsp.h"
#include "device.h"
#include "data_link.h"
#include <filesystem>

namespace soundlink {

inline cfg_vec make_link_configs(device dev, const channel_vec& channels) {
    cfg_vec configs;
    configs.reserve(channels.size());
    for (const channel_info& ch : channels) {
        configs.emplace_back(dev, ch.fc, BW(ch.bandwidth), code_rate(ch.code_rate), modulation(ch.mod));
    }
    return configs;
}

struct txdata_ch:tx_channel, inspectable{
    module_snapshot inspect() const override;
    txdata_ch(const link_cfg& _cfg, TxMessage& message, u8 mask)
        : sch(), cfg(_cfg),
          data(&sch, "data", 512),
          coded(&sch, "coded", 512),
          mixed(&sch, "mixed", 512),
          bits(&sch, "bits", 4096),
          grid(&sch, "grid", 4096),
          wave(&sch, "wave", 8192),
          audio(&sch, "audio", 24000),
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

    void run() override {
        if (output.readable() >= TARGET_FILL) return;
        while(true){
            sch.run();
            if (output.readable() >= TARGET_FILL) break;
            source.grant_write();
            sch.step();
            if (output.readable()>= TARGET_FILL) break;   
        }

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


struct txctrl_ch:tx_channel, inspectable {
    module_snapshot inspect() const override;
    txctrl_ch(const link_cfg& _cfg, TxctrlQueue& controls, u8 mask)
        : sch(), cfg(_cfg),
          data(&sch, "data", 256),
          coded(&sch, "coded", 256),
          mixed(&sch, "mixed", 256),
          bits(&sch, "bits", 4096),
          grid(&sch, "grid", 4096),
          wave(&sch, "wave", 8192),
          audio(&sch, "audio", 24000),
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

    void run() override {
        if (output.readable() >= TARGET_FILL) return;
        while(true){
            sch.run();
            if (output.readable() >= TARGET_FILL) break;
            source.grant_write();
            sch.step();
            if (output.readable()>= TARGET_FILL) break;   
        }
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
    uint64_t origin;
    bool valid = false;
    rx_stamp(const link_cfg& cfg, uint64_t n)
        :origin(n), ratio(48000.0 / cfg.sr), offset(0) {
        // Combined DUC and DDC sample alignment, in 48 kHz audio samples.
        switch (cfg.bw) {
        case BW1_4: offset = 325.0; break;
        case BW3:   offset = 200.0; break;
        case BW5:   offset = 62.5; break;
        case BW10:  offset = 30.75; break;
        }
    }
    void update(const pss_tracker& pt, const slot_tracker& st) {
        if (st.processed_grids == checked_grid)
            return;
        checked_grid = st.processed_grids;
        if (!pt.pss_valid || !st.successful_last)
            return;

        if (pt.pss_grid >= st.processed_grids) {
            const u32 delta = (pt.pss_grid - st.processed_grids) & 7u;
            ft.slot_number = (st.sltn + delta) & 7u;
        } else {
            const u32 delta = (st.processed_grids - pt.pss_grid) & 7u;
            ft.slot_number = (st.sltn + 8 - delta) & 7u;
        }
        ft.sample = origin + uint64_t(pt.pss_sample * ratio + offset);
        valid = true;
    }
    void reset() {
        checked_grid = 0;
        valid = false;
    }
private:
    double ratio;
    double offset;
    u32 checked_grid = 0;
};
struct rxdata_ch:rx_channel, inspectable {
    module_snapshot inspect() const override;

    rxdata_ch(const link_cfg& _cfg, RxMessage& message,
         TxctrlQueue& controls, u8 mask, uint64_t origin = 0)
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
          ofdm(&sch, wave, grid, cfg, mask),
          mapper(&sch, grid, bits, cfg, mask),
          conv(&sch, bits, mixed, cfg.cr),
          shuffle(&sch, mixed, coded),
          bch(&sch, coded, data),
          sink(&sch, data, message, controls),
          stamp(cfg, origin) {}

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
        stamp.update(ofdm.pt, mapper.st);
        ofdm.set_slot(mapper.st);
        if(ofdm.failed() || mapper.failed()){
            ofdm.reset();
            mapper.reset();
            stamp.reset();
        }
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


struct rxctrl_ch:rx_channel, inspectable {
    module_snapshot inspect() const override;
    rxctrl_ch(const link_cfg& _cfg, rxCtrlQueue& controls, u8 mask, uint64_t origin = 0)
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
          ofdm(&sch, wave, grid, cfg, mask),
          mapper(&sch, grid, bits, cfg, mask),
          conv(&sch, bits, mixed, cfg.cr),
          shuffle(&sch, mixed, coded),
          bch(&sch, coded, data),
          sink(&sch, data, controls),
          stamp(cfg, origin) {}

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
        stamp.update(ofdm.pt, mapper.st);
        ofdm.set_slot(mapper.st);
        if(ofdm.failed() || mapper.failed()){
            ofdm.reset();
            mapper.reset();
            stamp.reset();
        }
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

inline const channel_vec h_cfg = {
    {4, FEC12, BW3, QPSK},
    {8, FEC12, BW3, QPSK},
    {12,FEC12, BW3, QPSK},
    {16,FEC12, BW3, QPSK}
};

// Fixed transfer configuration; center frequencies are in kHz.
inline const channel_vec fixed_data_cfg = {
    {3, FEC34, BW3, QPSK},
    {9, FEC34, BW5, QPSK},
    {14,FEC34, BW3, QAM16}
};
inline const channel_vec fixed_ctrl_cfg = {
    {16, FEC12, BW3, QPSK}
};

struct sender : inspectable {
    module_snapshot inspect() const override;
    enum class state {startup, handshake, transfer, close };
    state current = state::startup;
    sender(const std::filesystem::path& path)
        : ad(h_TDD),msg(path, 160), txctrl(20), rxctrl(20) {}
    void start(){
        if (current != state::startup) return;
        cfg_vec ctrl = make_link_configs(device::sender, h_cfg);
        create_tx({}, ctrl, h_TDD, 0);
        create_rx(ctrl, u8(~h_TDD));
        txctrl.push(ctrl_factory::hello(msg.metadata));
        ad.play();
        while (ma_pcm_rb_available_read(&ad.speaker) < TARGET_FILL) {
            for (auto* ch : tx) ch->run();
            ad.mix_audio(tx);
        }
        ad.start();
        current = state::handshake;
    }
    void step() {
        if (current == state::startup || finished) return;
        if(acked_timing && calculate_time(rx, timing_report)){
            if (txctrl.push(ctrl_factory::timing_adjustment(timing_report)))
                acked_timing = false;
        }
        switch (current) {
        case state::startup:
            return;
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
                        acked_hello = true;
                    // Keep hello transmitting until its replacement is queued.
                    break;
                }
                case ctrl_type::config: {
                    read_config(packet, data_cfg, ctrl_cfg);
                    txctrl.remove(hello_sequence);
                    txctrl.push(ctrl_factory::config_ack(packet.sequence));
                    // ready for tansission mode
                    cfg_vec data = make_link_configs(device::sender, data_cfg);
                    cfg_vec tx_ctrl = make_link_configs(device::sender, ctrl_cfg);
                    cfg_vec rx_ctrl = make_link_configs(device::sender, h_cfg);

                    u8 next_slot = ad.update_mask(t_TDD);
                    create_tx(data, tx_ctrl, t_TDD, next_slot);
                    create_rx(rx_ctrl, u8(~t_TDD));

                    for (tx_channel* ch : tx)
                        ch->run();
                    acked_cfg = true;
                    current = state::transfer;
                    
                    break;
                }
                case ctrl_type::timing_ack: {
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
                    for (uint16_t segment : read_ack(packet))
                        msg.acknowledge(segment);
                    break;
                }
                case ctrl_type::timing_ack: {
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
                    ad.stop();
                    finished = true;
                    break;
                }
                case ctrl_type::timing_ack: {
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
    void create_tx(const cfg_vec& data, const cfg_vec& ctrl,
        u8 mask, u8 starting_slot) {
        for (tx_channel* ch : tx) delete ch;
        tx.clear();

        for (const link_cfg& cfg : data) {
            txdata_ch* ch = new txdata_ch(cfg, msg, mask);
            ch->set_sltn(starting_slot);
            tx.push_back(ch);
        }

        for (const link_cfg& cfg : ctrl) {
            txctrl_ch* ch = new txctrl_ch(cfg, txctrl, mask);
            ch->set_sltn(starting_slot);
            tx.push_back(ch);
        }
    }
    void create_rx(const cfg_vec& ctrl, u8 mask) {
        uint64_t origin = ad.delivered2rx();

        for (rx_channel* ch : rx) delete ch;
        rx.clear();

        for (const link_cfg& cfg : ctrl) {
            rx.push_back(new rxctrl_ch(cfg, rxctrl, mask, origin));
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
    channel_vec data_cfg, ctrl_cfg;
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
struct receiver : inspectable {
    module_snapshot inspect() const override;
    enum class state {startup, handshake, transfer, close };
    state current = state::startup;

    receiver(const std::filesystem::path& path)
        : ad(u8(~h_TDD)),output_path(path), txctrl(20), rxctrl(20) {}
    void start(){
        if (current != state::startup) return;

        cfg_vec ctrl = make_link_configs(device::receiver, h_cfg);
        create_rx({}, ctrl, h_TDD);
        while (ma_pcm_rb_available_read(&ad.speaker) < TARGET_FILL)
            ad.mix_audio(tx);
        ad.start();
        current = state::handshake;

    }

    void step() {
        if (current == state::startup || finished) return;

        switch (current) {
        case state::startup:
            return;
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
                    u8 next_slot = ad.calibrate(&timing);
                    cfg_vec ctrl = make_link_configs(device::receiver, h_cfg);
                    create_tx(ctrl, u8(~h_TDD), next_slot);;
                    ad.arm();
                    tx_init = true;
                }
            }

            for (tx_channel* ch : tx) ch->run();
            ad.mix_audio(tx);

            if (pre_defined_conf && tx_init && msg) {
                txctrl.push(ctrl_factory::config(fixed_data_cfg, fixed_ctrl_cfg));

                cfg_vec data = make_link_configs(device::receiver, fixed_data_cfg);
                cfg_vec ctrl = make_link_configs(device::receiver, fixed_ctrl_cfg);
                create_rx(data, ctrl, t_TDD);
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
                u8 next_slot = ad.update_mask(u8(~t_TDD));

                cfg_vec ctrl = make_link_configs(device::receiver, h_cfg);
                create_tx(ctrl, u8(~t_TDD), next_slot);
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
                    ad.stop_after(8);
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

            while (!rxctrl.empty())
                rxctrl.pop();

            for (tx_channel* ch : tx) ch->run();
            ad.mix_audio(tx);
            if (ad.is_done()) {
                finished = true;
                return;
            }
            break;
        }
        }

        if (timing_sequence && !ad.get_adj()) {
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
        if (timing_sequence || ad.get_adj())
            return;

        int16_t adjustment = (int16_t)(
            (uint16_t(packet.fields[0]) << 8) | packet.fields[1]);
        if (adjustment < -1000 || adjustment > 1000) adjustment = 0;

        timing_sequence = packet.sequence;
        ad.set_adj(adjustment);
    }

    void create_tx(const cfg_vec& ctrl, u8 mask, u8 starting_slot) {
        for (tx_channel* ch : tx) delete ch;
        tx.clear();

        for (const link_cfg& cfg : ctrl) {
            txctrl_ch* ch = new txctrl_ch(cfg, txctrl, mask);
            ch->set_sltn(starting_slot);
            tx.push_back(ch);
        }

        for (tx_channel* ch : tx)
            ch->run();
    }

    void create_rx(const cfg_vec& data, const cfg_vec& ctrl, u8 mask) {
        uint64_t origin = ad.delivered2rx();

        for (rx_channel* ch : rx) delete ch;
        rx.clear();

        for (const link_cfg& cfg : data) {
            rx.push_back(new rxdata_ch(cfg, *msg, txctrl, mask, origin));
        }

        for (const link_cfg& cfg : ctrl) {
            rx.push_back(new rxctrl_ch(cfg, rxctrl, mask, origin));
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
