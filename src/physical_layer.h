#ifndef SOUNDLINK_PHYSICAL_LAYER_H
#define SOUNDLINK_PHYSICAL_LAYER_H

#include <vector>
#include "framework.h"



namespace soundlink{

    constexpr u16 NSYM_PER_RB = 6;

    typedef cf32 (*mod_t)(u8);
    // defines
    enum class BW{
        BW1_4, BW3, BW5, BW10
    };
    enum class modulation{
        BPSK = 1, QPSK = 2, QAM16 = 4, QAM64 = 6
    };
    enum class RE_type :u8{
        D, plt, PSS, ctrl, unused
    };

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
    struct pRB{
        modulation m;
        pRB(u16 _idx, u16 _nRB):idx(_idx), nRB(_nRB),
        m(modulation::BPSK){
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
        const u16 idx;
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

    struct modulator {
        uint64_t pending = 0;
        u8 available_bits = 0;
        inline static constexpr mod_t maps[7] = {nullptr, mapBPSK, mapQPSK,
            nullptr, mapQAM16, nullptr, mapQAM64};

        size_t fill(pRB& rb, const u8* input, cf32* grid) {
            u8 bits = static_cast<u8>(rb.m);
            u8 mask = (1u << bits) - 1u;
            size_t consumed = 0;
            u16 index;

            while (rb.nextData(index)) {
                if (available_bits < bits) {
                    pending |= uint64_t(input[consumed++])<< available_bits;
                    available_bits += 8;
                }
                grid[index] = maps[bits](pending & mask);
                pending >>= bits;
                available_bits -= bits;
            }
            return consumed;
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
            for (int i = 0; i < warm_up; ++i)
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
            for (int i = 0; i < warm_up; ++i)
                lfsr();
        }
    };
    inline void pilot_fill(slot& sl, gold& gen, cf32* grid) {
        for (pRB& rb : sl.rb_vec) {
            u16 bits = gen.step();
            for (u8 i = 0; i < 6; ++i) {
                grid[rb(plt_sc[i], plt_sym[i])] = mapQPSK(bits & 0x3u);
                bits >>= 2;
            }
        }
    }

    // synchronization section

    // zadoff-chu 62 length sequence with root: 25
    inline void makePSS(cf32* out) {
        constexpr u8 root = 25;
        constexpr float pi = 3.141592;

        for (u8 n = 0; n < 62; ++n) {
            u8 k = (n < 31) ? n : n + 1;
            float phase = -pi * root * k * (k + 1) / 63.0;

            out[n] = cf32(std::cos(phase), std::sin(phase));
        }
    }
    inline void pss_fill(slot& sl, const cf32* pss, cf32* grid) {
        if (sl.sltnmb != 0)
            return;

        u16 nRB = sl.rb_vec.size();
        u16 first = (nRB * 12 - 72) / 2;

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
    struct slotMapper:runnable{
        slotMapper(scheduler* sch, pipebuf<u8>& _in, pipebuf<cf32>& _grid, const link_cfg& _cfg):
        runnable(sch, "slotMapper"), cfg(_cfg), primary(0, _cfg.nRB),
        secondary(1, _cfg.nRB), in(_in), grid(_grid, _cfg.nSC * _cfg.nSym){
            makePSS(pss);
        }
        void run() override{
            slot& sl = slot_number == 0? primary:secondary;
            if(sl.capacity() > 8* in.readable() + mapper.available_bits ||
                grid.writable() < cfg.nSC * cfg.nSym)
                return;
            pss_fill(sl, pss, grid.wr());
            pilot_fill(sl, pltGen, grid.wr());
            /*
            management fill: TBD
            */
           for(pRB& _rb: sl.rb_vec){
            size_t consumed = mapper.fill(_rb, in.rd(), grid.wr());
            in.read(consumed);
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
        modulator mapper;
        gold pltGen;
        cf32 pss[62];
        pipereader<u8> in;
        pipewriter<cf32> grid;
        
    };

}
#endif //SOUNDLINK_PHYSICAL_LAYER_H
