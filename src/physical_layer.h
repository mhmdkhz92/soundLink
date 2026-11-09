#ifndef SOUNDLINK_PHYSICAL_LAYER_H
#define SOUNDLINK_PHYSICAL_LAYER_H

#include "framework.h"
#include "math.h"
#include "bch.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace soundlink{

typedef cf32 (*mod_t)(u8);

enum class RE_type :u8{
    D, plt, PSS, ctrl, unused};

// Abstraction of Physical Resource Blocks and slots
static constexpr u8 plt_sc[6]  = {2, 6, 9, 3, 7, 10};
static constexpr u8 plt_sym[6] = {1, 1, 1, 4, 4, 4};
struct pRB{
    const u16 idx;
    const u16 nRB;
    modulation m;
    pRB(u16 _idx, u16 _nRB, modulation _m)
    :idx(_idx), nRB(_nRB), m(_m){
        for(size_t i = 0; i < 12*nSym; ++i)
            layout[i] = RE_type::D;
        for(size_t i = 0; i < 6; ++i)
            set_label(plt_sc[i], plt_sym[i], RE_type::plt);
    }
    u16 operator()(u16 s, u16 t) const{
        return 12* (nRB  * t + idx) + s;
    }
    u16 operator()(u16 d) const{
        u16 t = d / 12;
        u16 s = d % 12;
        return (*this)(s, t);
    }
    bool nextData(u16& index) {
        while (data_idx < 12 * nSym) {
            const u16 local = data_idx++;

            if (layout[local] == RE_type::D) {
                index = (*this)(local);
                return true;
            }
        }
        return false;
    }
    void set_label(u16 d, RE_type type){
        if (layout[d]!=RE_type::D && layout[d] != type)
            fail("Resource Element override");
        if(layout[d] == RE_type::D && type != RE_type::D){
            dataRE_num--;
        layout[d] = type;
        }
    }
    void set_label(u16 s, u16 t, RE_type type){
        set_label(12 * t + s, type);
    }
    RE_type get_label(u16 d) const{
        return layout[d];
    }
    RE_type get_label(u16 s, u16 t) const{
        return get_label(12 * t + s);
    }
    void resetData() {
        data_idx = 0;
    }
    u16 capacity() const{
        return dataRE_num * static_cast<u16>(m);
    }
private:
    static const u16 nSym = NSYM_PER_RB;
    RE_type layout[12 * nSym];
    u16 data_idx = 0;
    u16 dataRE_num = 12 * nSym;

};

struct slot{
    u16 sltnmb;
    std::vector<pRB> rb_vec;
    slot(u16 _sltnmb, u16 _nRB, modulation _m)
    :sltnmb(_sltnmb){
        rb_vec.reserve(_nRB);  
        for (u16 idx = 0; idx < _nRB; ++idx)
            rb_vec.push_back(pRB(idx, _nRB, _m));

        pRB& rb = rb_vec[_nRB / 2 + (_nRB & 1u)];
        for (u16 i = 0; i < 12; ++i) {
            if (rb.get_label(i, 1) != RE_type::plt)
                rb.set_label(i, 1, RE_type::ctrl);
        }

        if(sltnmb !=0)return;
        u16 first = (_nRB * 12 - 72) / 2;
        for (u8 i = 0; i < 72; ++i) {
            u16 k = first + i;
            pRB& rb = rb_vec[k / 12];
            u8 sc = k % 12;
            if (i < 5 || i >= 67) 
                rb.set_label(sc, 0, RE_type::unused);
            else 
                rb.set_label(sc, 0, RE_type::PSS);
        }        
    }
    void reset(){
        for (pRB& rb : rb_vec)
            rb.resetData();
    }
    u16 capacity(){
        u16 c = 0;
        for(pRB& rb: rb_vec)
            c+= rb.capacity();
        return c;
    }
};

//mapping section
static constexpr float QAM16_NORM = 0.31622776602f;
static constexpr float QAM64_NORM = 0.15430334996f;
static constexpr float QPSK_NORM  = 0.70710678118f;

static constexpr cf32 lut_BPSK[2] = {
    {-1.0f, 0.0f},
    {+1.0f, 0.0f}
};
static constexpr cf32 lut_QPSK[4] = {
    {-QPSK_NORM, -QPSK_NORM},   // 00
    {-QPSK_NORM, +QPSK_NORM},   // 01
    {+QPSK_NORM, -QPSK_NORM},   // 10
    {+QPSK_NORM, +QPSK_NORM}    // 11
};
static constexpr float lut_PAM4[4] = {
    -3.0f * QAM16_NORM,
    -1.0f * QAM16_NORM,
    +3.0f * QAM16_NORM,
    +1.0f * QAM16_NORM
};
static constexpr float lut_PAM8[8] = {
    -7.0f * QAM64_NORM,
    -5.0f * QAM64_NORM,
    -1.0f * QAM64_NORM,
    -3.0f * QAM64_NORM,
    +7.0f * QAM64_NORM,
    +5.0f * QAM64_NORM,
    +1.0f * QAM64_NORM,
    +3.0f * QAM64_NORM
};
inline cf32 mapBPSK(u8 x){
    return lut_BPSK[x & 1];
}

inline cf32 mapQPSK(u8 x){
    return lut_QPSK[x&0x3];
}

inline cf32 mapQAM16(u8 x){
u8 i = (x >> 2) & 0x3;
u8 q =  x       & 0x3;
return { lut_PAM4[i], lut_PAM4[q] };
}

inline cf32 mapQAM64(u8 x){
u8 i = (x >> 3) & 0x7;
u8 q =  x       & 0x7;
return { lut_PAM8[i], lut_PAM8[q] };
}

struct LLRtab{
    LLRtab(){
        float step = 2*d_max/(float)dim;
        float d = -d_max + step/2;
        for (int y = 0; y < dim; ++y){
            bpsk[y] = 4.0f * d;
            qpsk[y] = 4.0f * QPSK_NORM * d;
            d += step;
        }
        qam16dist();
        qam64dist();
    }
    void eval(cf32 y, int m, u8* llr){
        u8 re = loc_index(y.real());
        u8 im = loc_index(y.imag());
        switch(m){
            case 1:
                llr[0] = llr_round(bpsk[re]);
                break;
            case 2:{
                llr[0] = llr_round(qpsk[re]);
                llr[1] = llr_round(qpsk[im]);
                break;
            }
            case 4:
                llr[0] = llr_round(qam16[0][re]);
                llr[1] = llr_round(qam16[1][re]);
                llr[2] = llr_round(qam16[0][im]);
                llr[3] = llr_round(qam16[1][im]);
                break;
            case 6:
                llr[0] = llr_round(qam64[0][re]);
                llr[1] = llr_round(qam64[1][re]);
                llr[2] = llr_round(qam64[2][re]);
                llr[3] = llr_round(qam64[0][im]);
                llr[4] = llr_round(qam64[1][im]);
                llr[5] = llr_round(qam64[2][im]);
                break;

        }
    }
    void setN0(float _N0){
        this->N0 = _N0;
    }
private:
    float llr_max = 6.0f;
    float d_max = 1.2f;
    static constexpr int dim = 64;
    float N0 = 1;
    float bpsk[dim];
    float qpsk[dim];
    float qam16[2][dim];
    float qam64[3][dim];

    void qam16dist(){
        float step = 2*d_max/(float)dim;
        for (int loc = 0; loc < 2; ++loc){
            float d = -d_max + step/2;
            for (int y = 0; y < dim; ++y){
                float dist = dmin(d, lut_PAM4, 4, loc, 0) - dmin(d, lut_PAM4, 4, loc, 1);
                qam16[loc][y] = dist;
                d += step;
            }
        }
    }
    void qam64dist(){
        float step = 2*d_max/(float)dim;
        for (int loc = 0; loc < 3; ++loc){
            float d = -d_max + step/2;
            for (int y = 0; y < dim; ++y){
                float dist = dmin(d, lut_PAM8, 8, loc, 0) - dmin(d, lut_PAM8, 8, loc, 1);
                qam64[loc][y] = dist;
                d += step;
            }
        }
    }
    float dmin(float d, const float* pam_lut, int lut_size,
        int loc, bool bit){
            float dmin = (2.0f * d_max) * (2.0f * d_max);
            for (int i = 0; i < lut_size; ++i){
                if(bool(i & (lut_size >> (loc + 1))) == bit){
                    float c = (pam_lut[i] - d)*(pam_lut[i] - d);
                    dmin = dmin < c?dmin:c;
                }
            }
            return dmin;
    }
    inline u8 llr_round(float delta){
        float llr = delta/N0;
        int llr_int = (int)(127/(2*llr_max)* (llr_max + llr));
        if(llr_int > 127)
            llr_int = 127;
        if (llr_int < 0)
            llr_int = 0;
        return (u8)llr_int;
    }
    inline u8 loc_index(float y){
        int t = (int)(dim * (y + d_max)/(2.0f * d_max));
        if (t >= dim)
            t = dim - 1;
        if (t < 0)
            t = 0;
        return (u8)(t);

    }
};
// pilot section
struct gold{
    u32 x1, x2;
    static constexpr u32 warm_up = 100;
    void lfsr(){
        // x1 step
        u32 fb = ((x1>>3) ^ x1) & 1;
        x1 = (x1 >> 1)|(fb << 30);
        // x2 step
        fb = ((x2>>3) ^ (x2>>2) ^ (x2>>1) ^ x2) & 1;
        x2 = (x2 >> 1)|(fb << 30);
    }
    gold():x1(1), x2(0x01234567u){
        // warm up
        for (u32 i = 0; i < warm_up; ++i)
            lfsr();
    }
    u16 step() {
        u16 out = 0;
        for (int i = 0; i < 16; ++i) {
            out |= ((x1 ^ x2) & 1) << i;
            lfsr();
        }
        return out;
    }
    void advance(int N){
        for (int i = 0; i < 16 * N; ++i) {
            lfsr();
        }
    }
    void reset(){
        x1 = 1;
        x2 = 0x01234567u;
        // warm up
        for (u32 i = 0; i < warm_up; ++i)
            lfsr();
    }
};
inline void pilot_fill(gold& gen, slot& sl, cv32 grid) {
    for (pRB& rb : sl.rb_vec) {
        u16 bits = gen.step();
        for (u8 i = 0; i < 6; ++i) {
            grid[rb(plt_sc[i], plt_sym[i])] = mapQPSK(bits & 0x3u);
            bits >>= 2;
        }
    }
}
// equalizer and modulation master
struct pRB_noise{
    std::vector<float> m2, m4;
    std::vector<float> value;
    int acc = 0;
    pRB_noise(size_t nRB)
    :m2(nRB, 0.0f), m4(nRB, 0.0f), value(nRB, 0.1f){}
    void update(slot& sl, cv32 grid){
        const float kappa[] = {0, 1, 1, 0, 1.32f, 0, 1.38095238f};
        for (size_t i = 0;  i < sl.rb_vec.size(); ++i){
            auto& rb = sl.rb_vec[i];
            float new_m2 = 0, new_m4 = 0;
            u16 index;
            u16 RE_count = rb.capacity() / static_cast<u16>(rb.m);
            
            while(rb.nextData(index)){
                float power = grid[index].re * grid[index].re + grid[index].im * grid[index].im;
                new_m2 += power;
                new_m4 += power * power;
            }
            new_m2/=RE_count;
            new_m4/=RE_count;

            float w = 0.2f + 0.8f / (acc + 1.0f);
            new_m2 = (1.0f - w) * m2[i] + w * new_m2;
            new_m4 = (1.0f - w) * m4[i] + w * new_m4;

            m2[i] = new_m2;
            m4[i] = new_m4;

            float d = 2.0f * new_m2 * new_m2 - new_m4;
            if (d < 0.0f)
                d = 0;

            int mod = static_cast<int>(rb.m);
            float signal = std::sqrt(d / (2 - kappa[mod]));
            if (signal >= new_m2) continue;
            value[i] = new_m2 - signal;
        }
        acc++;
        sl.reset();
    }
    void reset(){
        m2.assign(m2.size(), 0.0f);
        m4.assign(m4.size(), 0.0f);
        value.assign(value.size(), 0.1f);
        acc = 0;
    }
};
struct equalizer{
public:
    equalizer(const link_cfg cfg):
    noise(cfg.nRB),
    h1(cfg.nSC), h4(cfg.nSC),
    nRB(cfg.nRB){};
    void process(slot& sl, cv32 grid){
 
        auto at = [nsc = 12 * nRB](size_t symbol, size_t sc) {
            return symbol * nsc + sc;
        };
        // fill pilots - exact locations for h1 and h4
        for (size_t rb = 0; rb < nRB; ++rb) {
            u16 bits = refgen.step();
            for (size_t p = 0; p < 6; ++p) {
                size_t sc = 12 * rb + plt_sc[p];
                cf32 reference = mapQPSK(bits & 3);
                cf32 received = grid[at(plt_sym[p], sc)];

                auto& h = p < 3 ? h1 : h4;
                h[sc] =  received * std::conj(reference);
                bits >>= 2;
            }
        }
        // vertical interpolation for symbols 1 and 4
        interp_v(h1, plt_sc);
        interp_v(h4, plt_sc + 3);

        // horizontal interpolation and appying to grid
        cf32 tap = 0;
        for (size_t sc = 0; sc < 12 * nRB; ++sc) {
            cf32 c1 = h1[sc];
            cf32 c4 = h4[sc];
            
            for (size_t symbol = 0; symbol < NSYM_PER_RB; ++symbol) {
                float t = (float(symbol) - 1.0f) / 3.0f;
                tap = c1 + t * (c4 - c1);
                cf32 val = grid[at(symbol, sc)];
                grid[at(symbol, sc)] = val/tap;
            }
        }
        noise.update(sl, grid);

    }
    void reset_pilots(){
        refgen.reset();

    }
    void skip_slots(int n){
        refgen.advance(n * nRB);
    }
    float noise_val(u16 rb_idx){
        return noise.value[rb_idx];
    }
    void reset() {
        refgen.reset();
        noise.reset();
    }

private:
    std::vector<cf32> h1, h4;
    u16 nRB;
    gold refgen;
    pRB_noise noise;
    void interp_v(std::vector<cf32>& h, const u8* positions){
        size_t p = 0;
        size_t down = positions[p++];
        size_t up   = positions[p++];
        auto freq = [nsc = h.size()](size_t sc) {
            return (float)sc + (sc >= nsc / 2 ? 1.0f : 0.0f);};
        cf32 slope = (h[up]-h[down])/cf32(freq(up) - freq(down));
        for (size_t sc = 0; sc < 12 * nRB; ++sc){
            if (sc == up && p < 3 * nRB){
                down = up;
                up = positions[p % 3] + 12 * (p / 3);
                ++p; slope = (h[up]-h[down])/cf32(freq(up) - freq(down));
            }
            cf32 temp = h[down] + slope * cf32(freq(sc) - freq(down)); 
            h[sc] = temp;           
        }
    }
};

// synchronization section

// zadoff-chu 62 length sequence with root, 31:sender, 32:receiver
inline void makePSS(cf32* out, device role) {
    int root = role == device::sender?31:32;
    for (u8 n = 0; n < 62; ++n) {
        u8 k = (n < 31) ? n : n + 1;
        float phase = -pi * root * k * (k + 1) / 63.0f;

        out[n] = cf32(std::cos(phase), std::sin(phase));
    }
}

inline void pss_fill(const cf32* pss, slot& sl, cv32 grid) {
    if (sl.sltnmb != 0)
        return;

    size_t nRB = sl.rb_vec.size();
    u16 first = u16((nRB * 12 - 72) / 2);

    for (u8 i = 0; i < 72; ++i) {
        u16 k = first + i;
        pRB& rb = sl.rb_vec[k / 12];
        u8 sc = k % 12;

        if (i < 5 || i >= 67)
            grid[rb(sc, 0)] = cf32(0.0f, 0.0f);
        else
            grid[rb(sc, 0)] = pss[i - 5];
    }
}

// control and management section
inline void ctrl_fill(u8 slot_number, const slot& sl, cv32 grid) {
    u8 code[2] = {u8(slot_number << 5), 0};
    bch_encode(code, bch15_5);
    u16 bits = (u16(code[0]) << 8) | code[1];

    size_t _nRB = sl.rb_vec.size();
    const pRB& rb = sl.rb_vec[ _nRB/ 2 + (_nRB & 1u)];
    for (u16 i = 0; i < 12; ++i) {
        if(rb.get_label(i, 1) == RE_type::ctrl){
            grid[rb(i, 1)] = mapQPSK(bits >> 14);
            bits <<= 2;
        }
    }
}

// Expects an equalized grid; returns the closest candidate without a confidence check.
inline bool ctrl_decode(u8& decoded, const slot& sl, cv32 grid) {
    float code_score[32];
    u8 best_slot = 0;
    size_t _nRB = sl.rb_vec.size();
    const pRB& rb = sl.rb_vec[ _nRB/ 2 + (_nRB & 1u)];
    for (u8 candidate = 0; candidate < 32; ++candidate) {
        u8 code[2] = {u8(candidate << 3), 0};
        bch_encode(code, bch15_5);
        u16 bits = (u16(code[0]) << 8) | code[1];
        float distance = 0;
        for (u16 i = 0; i < 12; ++i) {
            if (rb.get_label(i, 1) == RE_type::ctrl) {
                distance += std::norm(cf32(grid[rb(i, 1)]) - mapQPSK(bits >> 14));
                bits <<= 2;
            }
        }
        code_score[candidate] = distance;
    }
    u8 best = (u8)soundlink::min_idx(code_score, 32);
    float best_value = code_score[best];
    code_score[best] = 1e8;
    float second_best = soundlink::min(code_score, 32);
    // all valid cases have 3 variable bits followed by two zero
    if(best & 3) return false; 
    if(best_value < 0.4 * second_best){
        decoded = u8(best>>2);
        return true;
    }
    return false;
}
// runnables:
struct slot_tracker{
    u8 sltn = 0;  // latest update 
    u32 processed_grids = 0;        // total grids processed
    u32 successful_total = 0;       // number of successful slt_number decodings
    bool successful_last = false;   // whether the last decoding was successful
    void update(bool status, u8 _sltn){
        processed_grids++;
        if(status){
            successful_last = true;
            successful_total++;
            sltn = _sltn;
        }
        else successful_last = false;

    }
};
struct pss_tracker {
    u32 published_grids = 0;
    u32 pss_grid = 0;
    uint64_t pss_sample = 0;
    bool pss_valid = false;
    void update() {
        ++published_grids;
        if (detected) {
            pss_grid = published_grids;
            pss_valid = true;
            detected = false;
        }
    }
    void new_pss_detected(uint64_t sample){
        detected = true;
        pss_sample = sample;
    }
private:
    bool detected = false;
};
struct slot_mapper:runnable{
slot_mapper(scheduler* sch, pipebuf<u8>& _in,
    pipebuf_c<float>& _grid, const link_cfg& _cfg, u8 mask):
    runnable(sch, "SLOT_MAP"),
    cfg(_cfg),
    primary(0, _cfg.nRB, cfg.mod),
    secondary(1, _cfg.nRB, cfg.mod),
    slot_mask(mask),
    in(_in),
    grid(_grid, _cfg.nSC * _cfg.nSym)
    {
        mod_t maps[] = {nullptr, mapBPSK, mapQPSK, nullptr, mapQAM16, nullptr, mapQAM64};
        map = maps[(int)cfg.mod];
        makePSS(pss, cfg.dev);
    }
    void run() override{
        bool active = (slot_mask >> (7-slot_number)) & 1u;
        bool prev_active = (slot_mask >> (8 - slot_number)) & 1u;
        if(active){
            slot& sl = !prev_active? primary:secondary;
            if(sl.capacity() > in.readable() || grid.writable() < cfg.nSC * cfg.nSym)
                return;
            pss_fill(pss, sl, grid.wr_c());
            pilot_fill(pltGen, sl, grid.wr_c());
            
            //management fill: TBD
            ctrl_fill(slot_number, sl, grid.wr_c());
            

            u16 index;
            u8 symbol;
            cv32 g = grid.wr_c();
            for (pRB& rb : sl.rb_vec) {
                u8 bits = static_cast<u8>(rb.m);
                while (rb.nextData(index)) {
                    in.read_bits(&symbol, bits);
                    g[index] = map(symbol >> (8 - bits));
                }
            }
            sl.reset();
        }
        else{
            if(grid.writable() < cfg.nSC * cfg.nSym)
                return;
            cv32 g = grid.wr_c();
            for (size_t i = 0; i < size_t(cfg.nSC) * cfg.nSym; ++i)
                g[i] = cf32(0.0f);
            pltGen.advance(cfg.nRB);
        }
        slot_number = (++slot_number) & 0x7;
        if (slot_number == 0) pltGen.reset();
        grid.written(cfg.nSC * cfg.nSym);
    }
    void set_sltn(u8 n){
        slot_number = n;
        pltGen.reset();
        pltGen.advance(cfg.nRB * n);
    }
private:
    const link_cfg& cfg;
    slot primary;
    slot secondary;
    u8 slot_number = 0;
    u8 slot_mask;
    gold pltGen;
    cf32 pss[62];
    pipereader_bit in;
    pipewriter<float> grid;
    mod_t map;
    
};

struct slot_demapper:runnable{
    slot_tracker st;
    slot_demapper(scheduler* sch, pipebuf_c<float>& _grid,
    pipebuf<u8>& _out, const link_cfg& _cfg, u8 mask):
    runnable(sch, "SLOT_DEMAP"),
    cfg(_cfg),
    primary(0, _cfg.nRB, cfg.mod),
    secondary(1, _cfg.nRB, cfg.mod),
    slot_mask(mask),
    failures(0),
    out(_out, int(22e3)),
    grid(_grid), eq(_cfg){}
    void acquire() {
        state = mode::acquiring;
    }
    void set_mask(u8 mask) {
        slot_mask = mask;
    }
    void run() override{
        const size_t count = cfg.nSC * cfg.nSym;
        if (grid.readable() < count) return;

        switch (state) {
        case mode::acquiring:
            if (primary.capacity() > out.writable()) return;
            if (!acquisition_mode()) {
                st.update(false, slot_number);
                grid.read(count);
                return;
            }
            state = mode::tracking;
            break;
        case mode::tracking:
            break;
        }
        bool active = (slot_mask >> (7-slot_number)) & 1u;
        bool prev_active = (slot_mask >> (8 - slot_number)) & 1u;
        if(active){
            slot& sl = !prev_active? primary:secondary;
            // capacity check
            if (sl.capacity() > out.writable())
                return;

            //pilot extraction - ref slot filling and phase compensation
            cv32 g = grid.rd_c();
            eq.process(sl, g);
            // management decoding
            st.update(sltn_check(), slot_number);
            // soft demapping
            u8 llr_val[6];
            u8* wr = out.wr();
            size_t consumed = 0;
            u16 index;
            for (pRB& rb : sl.rb_vec) {
                u8 m = static_cast<u8>(rb.m);
                llr.setN0(eq.noise_val(rb.idx));
                while (rb.nextData(index)) {
                    llr.eval(g[index], m, llr_val);
                    for (int w = 0; w < m; w++){
                        wr[consumed++] = llr_val[w];
                    }        
                }
            }
            out.written(consumed);
            sl.reset();
            constellation(sl);
        }else{
            eq.skip_slots(1);
            st.update(false, slot_number);
        } 
        slot_number = (slot_number + 1) & 0x7;
        if (slot_number == 0) eq.reset_pilots();
        grid.read(count);
        
    }

    bool acquisition_mode(){
        std::vector<float> scores(8, 1e6f);
        size_t gsize = cfg.nSC * cfg.nSym;
        cv32 g = grid.rd_c();
        std::vector<float> re(gsize), im(gsize);
        cv32 trial{re.data(), im.data()};
        equalizer _eq(cfg);

        u8 h_number = 0;
        for(int i = 0; i < 8; ++i){
            if(((slot_mask >> (7 - i)) & 1u) == 0)continue;
        _eq.reset();
        _eq.skip_slots(i);
        std::memcpy(trial.re, g.re, gsize * sizeof(float));
        std::memcpy(trial.im, g.im, gsize * sizeof(float));
        _eq.process(primary, trial);
        bool decode_flag = ctrl_decode(h_number, primary, trial);
        if (!decode_flag) continue;
        if (h_number!=i) continue;

        float mean_noise = 0;
        for(u8 j = 0; j < cfg.nRB; ++j)
            mean_noise += _eq.noise_val(j);
        scores[i] = mean_noise/(float)cfg.nRB;  
        }
    u8 best = 0;
    for(u8 j = 0; j < 8; ++j){
        if (scores[j] < scores[best]) best = j;
    }
    if (scores[best] > 0.25) return false; // huristic
    slot_number = best;
    eq.reset();
    eq.skip_slots(best);
    return true;
    }
    bool sltn_check(){
        u8 hypot = 0;
        bool decode_flag = ctrl_decode(hypot, primary, grid.rd_c());
        if (!decode_flag || slot_number != hypot){
            failures++;
            return false;
        }
            failures = 0;
            return true;
    }
    bool failed(){
        return failures > 3 || (state == mode::acquiring && st.processed_grids >= 8);
    }
    void reset(){
        grid.read(grid.readable());
        st = slot_tracker{};
        failures = 0;
        state = mode::acquiring;
    }
    bool snr_estimate(float& db) {
        if (state != mode::tracking) return false;

        float noise = 0.0;
        unsigned n = 0;
        for (u16 rb = 0; rb < cfg.nRB; ++rb) {
            const float n0 = eq.noise_val(rb);
            if (!std::isfinite(n0) || n0 <= 0.0f) continue;
            noise += n0;
            ++n;
        }
        if (n < 5) return false;
        db = (float)(-10.0 * std::log10(noise / n));
        return true;
    }
    void constellation(slot& sl){
        if(state == mode::acquiring || !pending.ready) return;
        u16 written = 0;
        cv32 g = grid.rd_c();
        u16 index;
        for(pRB&rb : sl.rb_vec){
            while(rb.nextData(index) && written < pending.capacity)
                pending.points[written++] = g[index];
            if(written == pending.capacity) break;
        }
        *pending.written = (size_t)written;
        sl.reset();
        auto ready = pending.ready;
        pending = {};
        ready();
    }
    void constellation_discard() {
        if (!pending.ready) return;
        *pending.written = 0;
        pending.ready();
    }

    sample_request<cf32> pending;
    ~slot_demapper() { constellation_discard(); }
private:
    enum class mode : u8 {
        acquiring,
        tracking
    };
    mode state = mode::acquiring;
    const link_cfg& cfg;
    slot primary;
    slot secondary;
    u8 slot_number = 0;
    u8 slot_mask;
    u8 failures;
    pipewriter<u8> out;
    pipereader<float> grid;
    equalizer eq;
    LLRtab llr;
};

// baseband signal generator
struct ofdm_modulator: runnable{
    ofdm_modulator(scheduler *sch,  pipebuf_c<float> &_in, pipebuf_c<float> &_out, 
    const link_cfg& _cfg):
    runnable(sch, "ofdm_modulator"),
    cfg(_cfg),
    fft_engine(_cfg.nFFT),
    in(_in), 
    out(_out, (cfg.nFFT + cfg.cp) * cfg.nSym){};
    void run() override{
        if(in.readable() < cfg.nSC * cfg.nSym ||
            out.writable() < (cfg.nFFT + cfg.cp) * cfg.nSym)
            return;
        

        cv32 symin  = in.rd_c();
        cv32 symout = out.wr_c();

        for (size_t i = 0; i < (cfg.nFFT + cfg.cp) * cfg.nSym; ++i)
            symout[i] = 0;

        size_t out_offset = (size_t)(cfg.cp + cfg.nFFT - cfg.nSC/2);
        size_t in_offset = (size_t)(cfg.nSC/2);
        
        size_t sym_index = 0;
        while(sym_index < cfg.nSym){
            for(size_t i = 0; i < cfg.nSC/2; ++i){
                symout[out_offset + i] = symin[i];
                symout[cfg.cp + 1 + i] = symin[in_offset + i];    
            }
            fft_engine.inverse(symout + cfg.cp);
            std::memmove(symout.re, symout.re + cfg.nFFT, sizeof(float) * cfg.cp);
            std::memmove(symout.im, symout.im + cfg.nFFT, sizeof(float) * cfg.cp);
            symin += cfg.nSC;
            symout += cfg.nFFT + cfg.cp;
            sym_index ++;
        }
        in.read(cfg.nSym * cfg.nSC);
        out.written(cfg.nSym * (cfg.nFFT + cfg.cp));
    }
private:
    const link_cfg& cfg;
    fft fft_engine;
    pipereader<float> in;
    pipewriter <float> out;


};
struct timesync{
    static constexpr size_t window = 4096;
    timesync(const link_cfg& cfg):
    fft_engine(window),
    pss{new float[window]{}, new float[window]{}},
    corr{new float[window]{}, new float[window]{}},
    npss(cfg.cp + cfg.nFFT){
        int root = cfg.dev == device::sender?32:31;
        for (int n = 0; n < 62; ++n) {
            int k = (n < 31) ? n : n + 1;
            int c = (n < 31) ? (cfg.nFFT - 31 + n):(n - 30);
            float phase = -pi * (float)(root * k * (k + 1)) / 63.0f;
            pss.re[c + cfg.cp] = std::cos(phase);
            pss.im[c + cfg.cp] = std::sin(phase);
        }
        fft symbol_fft(cfg.nFFT);
        symbol_fft.inverse(pss + cfg.cp);
        for (size_t i = 0; i < cfg.cp; ++i)
            pss[i] = pss[cfg.nFFT + i];
        fft_engine.forward(pss);
        pssEnergy = 0;
        for (size_t i = 0; i < window; ++i){
            pss.im[i] = -pss.im[i];
            pssEnergy += pss.im[i]*pss.im[i] + pss.re[i] * pss.re[i];
        }
        pssEnergy /= window;
        th = 0.3f * 62.0f / (cfg.nSC - 10.0f);
    }
    ~timesync(){
        delete[] pss.re;delete[] pss.im;
        delete[] corr.re;delete[] corr.im;
    }
    int sync(cv32 sig, size_t n){
        // fill correlation container
        size_t m = std::min(n, window);
        size_t valid = m - npss + 1;
        for(size_t i = 0; i < window; ++i){
            corr[i] = i < m?sig[i]:cf32(0);
        }
        // compute fft,
        fft_engine.forward(corr);
        product_c(corr, pss, window);
        fft_engine.inverse(corr);
        // normalize
        float sigEnergy = 0;

        auto abs = [](complex_ref<float> x) {
            return x.re * x.re + x.im * x.im;
        };
        for (size_t i = 0; i < npss; ++i){
            sigEnergy += abs(sig[i]);
        }
        float peak = 0;
        size_t peak_idx = 0;
        for (size_t i = 0; i < valid; ++i){
            float c = abs(corr[i]);
            c /= (sigEnergy * pssEnergy + 1e-7f);
            if (i + 1 < valid)
                sigEnergy += abs(sig[i + npss]) - abs(sig[i]);
            if (c > peak){
                peak = c;
                peak_idx = i;
            }
        }
        if (peak > th) return (int)peak_idx;
        return -1;
    }
private:
    float th;
    fft fft_engine;
    cv32 pss;
    cv32 corr;
    size_t npss;
    float pssEnergy;
};
struct freqsync{
    freqsync(const link_cfg& cfg):
    osc(1), fr(0.005, 0.1f), sr(cfg.sr), nfft(cfg.nFFT), cp(cfg.cp){
        step = 1;
    };
    void estimate(cv32 ofdm){
        float re = 0;
        float im = 0;
        float* re_ofdm = ofdm.re;
        float* im_ofdm = ofdm.im;
        for (int i = 0; i < cp; ++i){
            re += re_ofdm[i] * re_ofdm[i + nfft] + im_ofdm[i] * im_ofdm[i + nfft];
            im += -re_ofdm[i] * im_ofdm[i + nfft] + im_ofdm[i] * re_ofdm[i + nfft];
        }
        float arg = std::arg(cf32(re, im));
        float _fr = -15.0f * arg/(2*pi);
        fr.update(_fr);
        float angle = -2 * pi * fr.value / sr;
        step = cf32(std::cos(angle), std::sin(angle));
        osc /= std::abs(osc);
    }
    void correct(cv32 ofdm){
        float re, im;
        float* re_ofdm = ofdm.re;
        float* im_ofdm = ofdm.im;
        for(int i = 0; i < cp + nfft; ++i){
            re = re_ofdm[i] * osc.real() - im_ofdm[i] * osc.imag();
            im = im_ofdm[i] * osc.real() + re_ofdm[i] * osc.imag();
            re_ofdm[i] = re; im_ofdm[i] = im;
            osc *= step;           
        }
    }
private:
    cf32 osc, step;
    kalman fr;
    float sr;
    int nfft, cp;
};
struct ofdm_demodulator: runnable{
    pss_tracker pt;
    bool synchronized() const { return state == mode::tracking;}
    void acquire() {state = mode::searching;}
    ofdm_demodulator(scheduler *sch, pipebuf_c<float> &_in,
    pipebuf_c<float> &_out, const link_cfg& _cfg, u8 mask):
    runnable(sch, "ofdm_demodulator"),
    preserve_length((int)(search_margin_audio_samples * _cfg.sr / 48000.0f)),
    cfg(_cfg),
    fft_engine(_cfg.nFFT),
    in(_in),
    out(_out, _cfg.nSC * _cfg.nSym),
    tsync(_cfg), fsync(_cfg),
    state(mode::searching),
    buf_re(cfg.nFFT + cfg.cp),
    buf_im(cfg.nFFT + cfg.cp),
    sltn(8), sltmask(mask),
    pss_failures(0){}

    void run() override{
        if (out.writable() < cfg.nSC * cfg.nSym) return;

        if(state == mode::searching) init_search();

        if(in.readable() < (size_t)(2 * preserve_length + (cfg.nFFT + cfg.cp) * cfg.nSym)
        || state == mode::searching) return;

        if(has_pss(sltn)) pss_check();

        cv32 symin = in.rd_c();
        if (active(sltn)) fsync.estimate(symin);   
        cv32 symout = out.wr_c();


        size_t in_offset = cfg.nFFT - cfg.nSC/2;
        size_t out_offset = cfg.nSC/2;

        for (int s = 0; s < cfg.nSym; ++s){
            const size_t bytes = (cfg.nFFT + cfg.cp) * sizeof(float);
            std::memcpy(buf_re.data(), symin.re, bytes);
            std::memcpy(buf_im.data(), symin.im, bytes);

            cv32 temp{buf_re.data(), buf_im.data()};

            fsync.correct(temp);
            fft_engine.forward(temp + cfg.cp);

            float* buffer_re = temp.re + cfg.cp;
            float* buffer_im = temp.im + cfg.cp;
            for(size_t i = 0; i < cfg.nSC/2; ++i){
                symout.re[i] = buffer_re[in_offset + i];
                symout.im[i] = buffer_im[in_offset + i];
                symout.re[out_offset + i] = buffer_re[1 + i];
                symout.im[out_offset + i] = buffer_im[1 + i];
            }
            symin += cfg.nFFT + cfg.cp;
            symout += cfg.nSC;
        }
        if (sltn < 8 && has_pss((sltn + 1) & 7u)){
            in.read(cfg.nSym * (cfg.nFFT + cfg.cp) - preserve_length);
        }else{
            in.read(cfg.nSym * (cfg.nFFT + cfg.cp));
        }
        out.written(cfg.nSym * cfg.nSC);
        if (sltn < 8) sltn = (sltn + 1) & 7u;
        pt.update();
    }
    void init_search(){
        while (state == mode::searching) {
            if (in.readable() < cfg.cp + cfg.nFFT)
                return;
            size_t searched = std::min(in.readable(), tsync.window);
            int t = tsync.sync(in.rd_c(), searched);
            if (t == -1) {
                in.read(searched - (cfg.nFFT + cfg.cp) + 1);
                continue;
            }
            pt.new_pss_detected(in.buf.total_read + t);
            in.read((size_t)t);
            state = mode::tracking;
        }
    }
    bool pss_check(){
        size_t searched =  cfg.cp + cfg.nFFT + 2 * preserve_length;
        int t = tsync.sync(in.rd_c(), searched);
        if (t == -1){
            pss_failures ++;
            in.read(preserve_length);
            return false;
        }
        pss_failures = 0;
        pt.new_pss_detected(in.buf.total_read + t);
        in.read((size_t)t);
        return true;
        }
    void set_slot(slot_tracker& st) {
        if (sltn < 8) return;  // Already synchronized
        if (!st.successful_last) return;

        const u32 elapsed = pt.published_grids - st.processed_grids;
        sltn = (st.sltn + elapsed + 1) & 7u;
    }
    bool active(u8 n) const {
        return n < 8 && ((sltmask >> (7 - n)) & 1u);
    }
    bool has_pss(u8 n) const {
        return active(n) && !active((n + 7) & 7u);
    }
    bool failed(){
        return pss_failures > 3;
    }
    void reset(){
        pt = pss_tracker{};
        sltn = 8;
        pss_failures = 0;
        state = mode::searching;
    }
private:
    enum class mode : u8 {
        searching,
        tracking
    };
    // 15.625 ms at 48 kHz; convert the same time margin to baseband samples.
    static constexpr int search_margin_audio_samples = 750;
    int preserve_length;

    const link_cfg& cfg;
    fft fft_engine;
    pipereader<float> in;
    pipewriter<float> out;
    timesync tsync;
    freqsync fsync;
    mode state;
    std::vector<float> buf_re, buf_im;
    
    u8 sltn;
    u8 sltmask;
    u8 pss_failures;
};
}
#endif //SOUNDLINK_PHYSICAL_LAYER_H
