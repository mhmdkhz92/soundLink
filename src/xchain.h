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


namespace soundlink {

constexpr uint8_t h_mask = 0b10101010;
constexpr uint8_t t_mask = 0b11111110;

struct profile{
    const bch_config& bch_cfg;
    size_t width;  // interleaver width
    size_t height; // interleaver height
    
};

static constexpr profile data_profile{bch127_106, 127, 24};
static constexpr profile ctrl_profile{bch63_36, 63, 8};

template<const profile& pr>
struct tx_chain {
    const link_cfg cfg;
    tx_chain(int fc_khz, BW bw, code_rate cr, u8 slt_pattern)
        : cfg(bw, slt_pattern),
          data(&sch, "input pipe", 8096),
          coded(&sch, "bch pipe", 8096),
          mixed(&sch, "interleaved pipe", 8096),
          bits(&sch, "convo enc pipe", 8096),
          grid(&sch, "slot pipe", 8096),
          wave(&sch, "ifft pipe", 8096),
          audio(&sch, "48kHz pipe", 24000),
          bch(&sch, data, coded),
          shuffle(&sch, coded, mixed),
          conv(&sch, mixed, bits, cr),
          mapper(&sch, bits, grid, cfg),
          ofdm(&sch, grid, wave, cfg),
          up(&sch, wave, audio, cfg){
        up.set_fc(fc_khz);
    }

    // Registered pointers and pipe references require a stable address.
    tx_chain(const tx_chain&) = delete;
    tx_chain& operator=(const tx_chain&) = delete;
    tx_chain(tx_chain&&) = delete;
    tx_chain& operator=(tx_chain&&) = delete;

    void step() { sch.step(); }
    
private:
    scheduler sch;
    pipebuf<u8> data;
    pipebuf<u8> coded;
    pipebuf<u8> mixed;
    pipebuf<u8> bits;
    pipebuf_c<float> grid;
    pipebuf_c<float> wave;
    pipebuf<float> audio;
    bch_encoder<pr.bch_cfg> bch;
    interleaver<pr.width, pr.height> shuffle;
    conv_encoder conv;
    slot_mapper mapper;
    ofdm_modulator ofdm;
    duc up;
};

template<const profile& pr>
struct rx_chain {
    const link_cfg cfg;
    rx_chain(int fc_khz, BW bw, code_rate cr, u8 slt_pattern)
        : cfg(bw, slt_pattern),
          audio(&sch, "48kHz pipe", 48000),
          wave(&sch, "baseband pipe", 8096),
          grid(&sch, "slot pipe", 8096),
          bits(&sch, "soft bits pipe", 24000),
          mixed(&sch, "decoded pipe", 8096),
          coded(&sch, "deinterleaved pipe", 8096),
          data(&sch, "output pipe", 8096),
          down(&sch, audio, wave, cfg),
          ofdm(&sch, wave, grid, cfg),
          mapper(&sch, grid, bits, cfg),
          conv(&sch, bits, mixed, cr),
          shuffle(&sch, mixed, coded),
          bch(&sch, coded, data) {
        down.set_fc(fc_khz);
    }

    // Registered pointers and pipe references require a stable address.
    rx_chain(const rx_chain&) = delete;
    rx_chain& operator=(const rx_chain&) = delete;
    rx_chain(rx_chain&&) = delete;
    rx_chain& operator=(rx_chain&&) = delete;

    void step() { sch.step(); }

private:
    scheduler sch;
    pipebuf<float> audio;
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
    deinterleaver<pr.width, pr.height> shuffle;
    bch_decoder<pr.bch_cfg> bch;
};
using tx_ctrl = tx_chain<ctrl_profile>;
using rx_ctrl = rx_chain<ctrl_profile>;
using tx_data = tx_chain<data_profile>;
using rx_data = rx_chain<data_profile>;

} // namespace soundlink

#endif
