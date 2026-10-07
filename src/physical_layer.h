#ifndef SOUNDLINK_PHYSICAL_LAYER_H
#define SOUNDLINK_PHYSICAL_LAYER_H

#include <vector>
#include <cmath>
#include "framework.h"



namespace soundlink{

constexpr u16 NSYM_PER_RB = 6;

typedef cf32 (*mod_t)(u8);
// defines
enum class BW{
    BW1_4, BW3, BW5, BW10};
enum class modulation{
    BPSK = 1, QPSK = 2, QAM16 = 4, QAM64 = 6};
enum class RE_type :u8{
    D, plt, PSS, ctrl, unused};

struct link_cfg{
    BW bw;                  // bandwidth in BW enum
    float bandwidth;        // bandwidth in Hz
    float sr;               // sample rate
    u16 nRB;                // number of resource blocks
    u16 nSC;                // number of active subcarriers
    u16 nFFT;               // number of FFT points
    u16 nSym;             // number of symbols ber slot
    u16 cp;                  // number of cyclic prefix points
    link_cfg(BW _bw): nSym(NSYM_PER_RB){
        switch (_bw){
        case BW::BW1_4:
            bw = BW::BW1_4; bandwidth = 1.4e3; nRB = 6; nFFT = 128;
            break;
        case BW::BW3:
            bw = BW::BW3; bandwidth = 3e3; nRB = 15; nFFT = 256;
            break;
        case BW::BW5:
            bw = BW::BW5; bandwidth = 5e3; nRB = 25; nFFT = 512;
            break;
        case BW::BW10:
            bw = BW::BW10; bandwidth = 10e3; nRB = 50; nFFT = 1024;
            break;
        }
        nSC = nRB * 12;
        sr = nFFT * 15;
        cp = nFFT / 4;
    }
};

// Abstraction of Physical Resource Blocks and slots
static constexpr u8 plt_sc[6]  = {2, 6, 9, 3, 7, 10};
static constexpr u8 plt_sym[6] = {1, 1, 1, 4, 4, 4};
struct rx_stats{
    float m2 = 0;
    float m4 = 0;
    float noise = 0.1f;
};
struct pRB{
    const u16 idx;
    modulation m = modulation::QPSK;
    pRB(u16 _idx, u16 _nRB): idx(_idx), nRB(_nRB){
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
    const u16 nRB;
    static const u16 nSym = NSYM_PER_RB;
    RE_type layout[12 * nSym];
    u16 data_idx = 0;
    u16 dataRE_num = 12 * nSym;

};
struct slot{
    u16 sltnmb;
    std::vector<pRB> rb_vec;
    slot(u16 _sltnmb, u16 _nRB): sltnmb(_sltnmb){
        rb_vec.reserve(_nRB);  
        for (u16 idx = 0; idx < _nRB; ++idx)
            rb_vec.push_back(pRB(idx, _nRB));
        if(sltnmb !=0)
            return;

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
    void reset(){
        x1 = 1;
        x2 = 0x01234567u;
        // warm up
        for (u32 i = 0; i < warm_up; ++i)
            lfsr();
    }
};
inline void pilot_fill(slot& sl, gold& gen, cv32 grid) {
    for (pRB& rb : sl.rb_vec) {
        u16 bits = gen.step();
        for (u8 i = 0; i < 6; ++i) {
            grid[rb(plt_sc[i], plt_sym[i])] = mapQPSK(bits & 0x3u);
            bits >>= 2;
        }
    }
}
// equalizer and modulation master
struct equalizer{
public:
    equalizer(const link_cfg cfg):
    stats(cfg.nRB), h1(cfg.nSC), h4(cfg.nSC),
    nRB(cfg.nRB){};

    void process(slot& sl, cv32 grid){
        u16 sltnmbr = sl.sltnmb;
        if(!sltnmbr) refgen.reset();
        if(sltnmbr && (sltnmbr - goldnmbr != 1)){
            refgen.reset();
            for (size_t i = 0; i < nRB * sltnmbr; ++i)
                refgen.step();
        }
        
        auto at = [nsc = 12 * nRB](size_t symbol, size_t sc) {
            return symbol * nsc + sc;
        };
        // fill pilots - exact locations for h1 and h4
        for (size_t rb = 0; rb < nRB; ++rb) {
            u16 bits = refgen.step();
            for (size_t p = 0; p < 6; ++p) {
                const size_t sc = 12 * rb + plt_sc[p];
                const cf32 reference = mapQPSK(bits & 3);
                const cf32 received = grid[at(plt_sym[p], sc)];

                auto& h = p < 3 ? h1 : h4;
                h[sc] = received * std::conj(reference);
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
        noise_update(sl, grid);
        goldnmbr = sltnmbr;

    }
    float noise_val(u16 rb_idx){
        return stats[rb_idx].noise;
    }

private:
    std::vector<rx_stats> stats;
    std::vector<cf32> h1, h4;
    size_t nRB;
    gold refgen;
    u16 goldnmbr = 255;
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
    void noise_update(slot& sl, cv32 grid){
        const float kappa[] = {0, 1, 1, 0, 1.32f, 0, 1.38095238f};
        for (size_t i = 0;  i < sl.rb_vec.size(); ++i){
            auto& rb = sl.rb_vec[i];
            u16 index;
            u16 c = rb.capacity() / static_cast<u16>(rb.m);
            float m2 = 0, m4 = 0;
            while(rb.nextData(index)){
                float power = grid[index].re * grid[index].re + grid[index].im * grid[index].im;
                m2 += power;
                m4 += power * power;
            }
            m2 /=c;m4/=c;
            if (!std::isfinite(m2) || !std::isfinite(m4))
                continue;
            if (stats[i].m2 == 0) stats[i].m2 = m2;
            if (stats[i].m4 == 0) stats[i].m4 = m4;
            stats[i].m2 = 0.8f * stats[i].m2 + 0.2f * m2;
            stats[i].m4 = 0.8f * stats[i].m4 + 0.2f * m4;
        
            m2 = stats[i].m2;
            m4 = stats[i].m4;
            float d = 2.0f * m2 * m2 - m4;
            if (d < 0.0f) continue;
            int mod = static_cast<int>(rb.m);
            float signal = std::sqrt(d / (2 - kappa[mod]));
            if (m2 < signal) continue;
            stats[i].noise = m2 - signal;
        }
        sl.reset();
    }
};

// synchronization section

// zadoff-chu 62 length sequence with root: 25
inline void makePSS(cf32* out) {
    constexpr u8 root = 25;
    for (u8 n = 0; n < 62; ++n) {
        u8 k = (n < 31) ? n : n + 1;
        float phase = -pi * root * k * (k + 1) / 63.0f;

        out[n] = cf32(std::cos(phase), std::sin(phase));
    }
}
inline void pss_fill(slot& sl, const cf32* pss, cv32 grid) {
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
// TBD

// runnables:
struct slot_mapper:runnable{
slot_mapper(scheduler* sch, pipebuf<u8>& _in,
    pipebuf_c<float>& _grid, const link_cfg& _cfg):
    runnable(sch, "SLOT_MAP"),
    cfg(_cfg),
    primary(0, _cfg.nRB),
    secondary(1, _cfg.nRB),
    in(_in),
    grid(_grid, _cfg.nSC * _cfg.nSym),
    maps{nullptr, mapBPSK, mapQPSK, nullptr, mapQAM16, nullptr, mapQAM64}{
        makePSS(pss);
    }
    void run() override{
        slot& sl = slot_number == 0? primary:secondary;
        if(sl.capacity() > in.readable() ||
            grid.writable() < cfg.nSC * cfg.nSym)
            return;
        pss_fill(sl, pss, grid.wr_c());
        pilot_fill(sl, pltGen, grid.wr_c());
        /*
        management fill: TBD
        */

        u16 index;
        u8 symbol;
        cv32 g = grid.wr_c();
        for (pRB& rb : sl.rb_vec) {
            u8 bits = static_cast<u8>(rb.m);
            while (rb.nextData(index)) {
                in.read_bits(&symbol, bits);
                g[index] = maps[bits](symbol >> (8 - bits));
            }
        }
        slot_number = (++slot_number) % 5;
        if (slot_number == 0)
        pltGen.reset();
        sl.reset();
        grid.written(cfg.nSC * cfg.nSym);
    }
private:
    const link_cfg& cfg;
    slot primary;
    slot secondary;
    u8 slot_number = 0;
    gold pltGen;
    cf32 pss[62];
    pipereader_bit in;
    pipewriter<float> grid;
    mod_t maps[7];
    
};

struct slot_demapper:runnable{
    slot_demapper(scheduler* sch, pipebuf_c<float>& _grid,
    pipebuf<u8>& _out, const link_cfg& _cfg):
    runnable(sch, "SLOT_DEMAP"),
    cfg(_cfg),
    primary(0, _cfg.nRB),
    secondary(1, _cfg.nRB),
    out(_out, int(22e3)),
    grid(_grid), eq(_cfg)
    {}
    void run() override{
        slot& sl = slot_number == 0? primary:secondary;
        sl.sltnmb = slot_number;
        // capacity check
        if (grid.readable() < cfg.nSC * cfg.nSym ||
            sl.capacity() > out.writable())
            return;

        
        //pilot extraction - ref slot filling and phase compensation
        cv32 g = grid.rd_c();
        eq.process(sl, g);

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
        slot_number = (++slot_number) % 5;
        sl.reset();
        grid.read(cfg.nSC * cfg.nSym);
    }

private:
    const link_cfg& cfg;
    slot primary;
    slot secondary;
    u8 slot_number = 0;
    pipewriter<u8> out;
    pipereader<float> grid;
    equalizer eq;
    LLRtab llr;
};
}
#endif //SOUNDLINK_PHYSICAL_LAYER_H
