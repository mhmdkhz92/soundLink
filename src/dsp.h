#ifndef SOUNDLINK_DSP_H
#define SOUNDLINK_DSP_H

#include <cstring>
#include "framework.h"
#include "math.h"
#include "physical_layer.h"

namespace soundlink{

    struct Kaiserparam{
        float beta;
        size_t numTaps;
    };

    inline void kaiserWin(float *y, Kaiserparam p){
        float x = 0;
        float *arg = new float[p.numTaps];
        for (size_t i = 0; i < p.numTaps; ++i){
            float t = std::pow(2 * x / (p.numTaps - 1) - 1, 2);
            arg[i] = p.beta * std::sqrtf(1 - t);
            x += 1;
        }
        besselI0(arg, y, p.numTaps);
        delete[] arg;
    }


    inline Kaiserparam estimateKaiser(float attenuation_dB, float f_trans){
        // Estimate beta.
        float beta = 0.0f;

        if (attenuation_dB > 50.0f)
            beta = 0.1102f * (attenuation_dB - 8.7f);

        else if (attenuation_dB >= 21.0f){
            float a = attenuation_dB - 21.0f;
            beta = 0.5842f * std::pow(a, 0.4f) + 0.07886f * a;
        }
        // Invalid transition width.
        if (f_trans <= 0.0f || f_trans >= 0.5f){
            return {beta, 0};
        }
        float deltaOmega = 2.0f * pi * f_trans;
        float order = std::ceil((attenuation_dB - 8.0f)/(2.285f * deltaOmega));
        if (order < 1.0f)
            order = 1.0f;

        size_t numTaps = static_cast<size_t>(order) + 1;

        // An odd number gives the FIR filter a center sample.
        if (numTaps % 2 == 0)
            ++numTaps;

        return {beta, numTaps};
    }

    inline void kaiserLpf(float* coefficients, float cutoff, Kaiserparam p){
    kaiserWin(coefficients, p);

    int center = static_cast<int>((p.numTaps - 1) / 2);
    float sum = 0.0f;

    for (size_t n = 0; n < p.numTaps; ++n) {
        int m = static_cast<int>(n) - center;
        float ideal;
        if (m == 0)
            ideal = 2.0f * cutoff;
        else
            ideal = std::sin(2.0f * pi * cutoff * m)/ (pi * m);
        coefficients[n] *= ideal;
        sum += coefficients[n];
    }

    // Unity gain at DC
    scale(coefficients, 1/sum, p.numTaps);
}

    // RESAMPLER runnable
    //uses normalized bandwidth: BW/Fs (two sided) and normalized cutoff: Fc/Fs
    template<typename T, int up, int down>
    struct resampler:runnable{
    public:
        resampler(scheduler *sch,  pipebuf<T> &_in, pipebuf<T> &_out,
             float n_cutoff, float n_transition)
        :runnable(sch, "resampler"),
        in(_in), out(_out){

            Kaiserparam p = estimateKaiser(60, n_transition/up);
            L_phase = (p.numTaps + up - 1) / up;

            float* h = new float[up*L_phase]{};
            filter = new float[up*L_phase]{};
            kaiserLpf(h, n_cutoff/up, p);
            scale(h, (float)up, p.numTaps);

            phase = new float*[up];
            for(int i = 0; i < up; i++){
                phase[i] = filter + (i* L_phase);
                for(int j = 0; j < L_phase; ++j){
                    filter[L_phase * (i + 1) - 1 - j] = h[i + up * j];
                }
            }
            delete[] h;
        }
        void run() override {
            T* curr = in.rd();
            T* end = curr + in.readable();

            while (end - curr >= L_phase &&
                end - curr >= (filter_phase + down) / up &&
                out.writable() > 0) {
                T res = 0;
                float* ph = phase[filter_phase];
                res = dot_product(curr, ph, L_phase);
                out.write(res);
                filter_phase += down;
                curr += filter_phase / up;
                filter_phase %= up;
            }

            in.read(curr - in.rd());
        }
        ~resampler(){
            delete[] phase;
            delete[] filter;
        }
    private:
        pipereader<T> in;
        pipewriter <T> out;
        float* filter = nullptr;
        unsigned L_phase = 0;
        unsigned filter_phase = 0;
        float** phase = nullptr;
    };

    // baseband signal generator
    struct ofdm_modulator: runnable{
        ofdm_modulator(scheduler *sch,  pipebuf<cf32> &_in, pipebuf<cf32> &_out, 
        const link_cfg& _cfg): runnable(sch, "ofdm_modulator"), 
        in(_in), out(_out, (cfg.nFFT + cfg.cp) * cfg.nSym), cfg(_cfg){
            fft_engines[0] = new fft<128>();fft_engines[1] = new fft<256>();
            fft_engines[2] = new fft<512>();fft_engines[3] = new fft<1024>();
            
        }
        void run() override{
            if(in.readable() < cfg.nSC * cfg.nSym ||
             out.writable() < (cfg.nFFT + cfg.cp) * cfg.nSym)
                return;
            

            cf32* symin  = in.rd();
            cf32* symout = out.wr();

            for (size_t i = 0; i < (cfg.nFFT + cfg.cp) * cfg.nSym; ++i)
                symout[i] = 0;

            size_t out_offset = (size_t)(cfg.cp + cfg.nFFT - cfg.nSC/2);
            size_t in_offset = (size_t)(cfg.nSC/2);
            size_t fft_index = static_cast<size_t>(cfg.bw);
            size_t sym_index = 0;
            while(sym_index < cfg.nSym){
                for(size_t i = 0; i < cfg.nSC/2; ++i){
                    symout[out_offset + i] = symin[i];
                    symout[cfg.cp + 1 + i] = symin[in_offset + i];    
                }
                fft_engines[fft_index]->inverse(symout + cfg.cp);
                std::memmove(symout, symout + cfg.nFFT, sizeof(cf32) * cfg.cp);
                symin +=  cfg.nSC;
                symout +=  cfg.nFFT + cfg.cp;
                sym_index ++;
            }
            in.read(cfg.nSym * cfg.nSC);
            out.written(cfg.nSym * (cfg.nFFT + cfg.cp));
        }
        ~ofdm_modulator(){
            delete fft_engines[0];
            delete fft_engines[1];
            delete fft_engines[2];
            delete fft_engines[3];
        }
    
    private:
        const link_cfg& cfg;
        fft_base* fft_engines[4];
        pipereader<cf32> in;
        pipewriter <cf32> out;


    };
    struct ofdm_demodulator: runnable{
        ofdm_demodulator(scheduler *sch, pipebuf<cf32> &_in,
        pipebuf<cf32> &_out, const link_cfg& _cfg):
        runnable(sch, "ofdm_demodulator"),
        cfg(_cfg), in(_in), out(_out, _cfg.nSC * _cfg.nSym){
            fft_engines[0] = new fft<128>();fft_engines[1] = new fft<256>();
            fft_engines[2] = new fft<512>();fft_engines[3] = new fft<1024>();

            buffer = new cf32[cfg.nFFT];
        }

        void run() override{
            if(in.readable() < (cfg.nFFT + cfg.cp) * cfg.nSym ||
            out.writable() < cfg.nSC * cfg.nSym)
                return;

            const cf32* symin = in.rd();
            cf32* symout = out.wr();

            size_t in_offset = cfg.nFFT - cfg.nSC/2;
            size_t out_offset = cfg.nSC/2;
            size_t fft_index = static_cast<size_t>(cfg.bw);
            size_t sym_index = 0;

            while(sym_index < cfg.nSym){
                std::memcpy(buffer, symin + cfg.cp,
                            sizeof(cf32) * cfg.nFFT);

                fft_engines[fft_index]->forward(buffer);

                for(size_t i = 0; i < cfg.nSC/2; ++i){
                    symout[i] = buffer[in_offset + i];
                    symout[out_offset + i] = buffer[1 + i];
                }

                symin += cfg.nFFT + cfg.cp;
                symout += cfg.nSC;
                sym_index++;
            }

            in.read(cfg.nSym * (cfg.nFFT + cfg.cp));
            out.written(cfg.nSym * cfg.nSC);
        }

        ~ofdm_demodulator(){
            delete fft_engines[0];
            delete fft_engines[1];
            delete fft_engines[2];
            delete fft_engines[3];
            delete[] buffer;
        }

    private:
        const link_cfg& cfg;
        fft_base* fft_engines[4];
        pipereader<cf32> in;
        pipewriter<cf32> out;
        cf32* buffer;
    };
    struct mixer: runnable{
        mixer(scheduler* sch, pipebuf<cf32>& _in, pipebuf<float>& _out):
        runnable(sch, "mixer"), in(_in), out(_out, 48), lut(fc, 48e3, winsize){
        }
        void run() override{
            while (in.readable() >= winsize &&
                out.writable() >= winsize) {
                const cf32* rd = in.rd();

                for (size_t i = 0; i < winsize; ++i) {
                    out.write(rd[i].real() * lut.cos[i] - rd[i].imag() * lut.sin[i]);
                }

                in.read(winsize);
            }
        }
        void set_fc(int fc_kHz) {
            if (fc_kHz < 2 || fc_kHz > 18)
                fail("wrong centre frequency was attempted");

            fc = fc_kHz * 1000.0f;
            lut.update(fc, 48000.0f);
    }
    private:
        float fc = 5e3;
        size_t winsize = 48;
        trianglut lut;
        pipereader<cf32> in;
        pipewriter<float> out;
        
    };
}


#endif
