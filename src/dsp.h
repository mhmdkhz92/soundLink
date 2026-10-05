#ifndef SOUNDLINK_DSP_H
#define SOUNDLINK_DSP_H

#include <cstring>
#include "framework.h"
#include "math.h"
#include "filter.h"
#include "physical_layer.h"

namespace soundlink{

// digital up converter runnable
struct duc:runnable{
public:
    duc(scheduler *sch,  pipebuf_c<float> &_in, pipebuf<float> &_out, const link_cfg& cfg)
    :runnable(sch, "duc"),
    in(_in), out(_out), lut(fc, 48e3, winsize), lut_idx(0){

        up = 25;
        down = 1u << static_cast<unsigned>(cfg.bw); // 1, 2, 4, 8
        // filter design
        const float cutoff = 0.5f / up;
        const float transition = (1.0f - cfg.bandwidth / cfg.sr) / up;

        // apply design and obtain taps
        Kaiserparam p = estimateKaiser(60, transition);
        L_phase = (p.numTaps + up - 1) / up;

        float* h = new float[up*L_phase]{};
        filter = new float[up*L_phase]{};
        kaiserLpf(h, cutoff, p);
        scale(h, (float)up, p.numTaps);


        phase = new float*[up];
        for(int i = 0; i < up; i++){
            phase[i] = filter + (i* L_phase);
            for(unsigned int j = 0; j < L_phase; ++j){
                filter[L_phase * (i + 1) - 1 - j] = h[i + up * j];
            }
        }
        delete[] h;
    }
    void run() override {
        const cv32 rd = in.rd_c();
        const size_t available = in.readable();
        const size_t capacity = out.writable();
        float* wr = out.wr();

        const size_t required = std::max(
            size_t(L_phase), size_t((down + up - 1) / up));

        size_t consumed = 0;
        size_t produced = 0;

        while (available - consumed >= required && produced < capacity) {
            const float* ph = phase[filter_phase];
            const float* input_re = rd.re + consumed;
            const float* input_im = rd.im + consumed;
            float re = 0.0f;
            float im = 0.0f;
            for (unsigned j = 0; j < L_phase; ++j) {
                const float h = ph[j];
                re += input_re[j] * h;
                im += input_im[j] * h;
            }
            wr[produced++] = re * lut.cos[lut_idx] - im * lut.sin[lut_idx];
            filter_phase += down;
            consumed += filter_phase / up;
            filter_phase %= up;
            if (++lut_idx == winsize)
                lut_idx = 0;
        }
        in.read(consumed);
        out.written(produced);
    }
    void set_fc(int fc_kHz) {
        if (fc_kHz < 2 || fc_kHz > 18)
            fail("wrong centre frequency was attempted");

        fc = fc_kHz * 1000.0f;
        lut.update(fc, 48000.0f);
}
    ~duc(){
        delete[] phase;
        delete[] filter;
    }
private:
    pipereader<float> in;
    pipewriter <float> out;

    // resampler attributes
    int up;
    int down;
    float* filter = nullptr;
    unsigned L_phase = 0;
    unsigned filter_phase = 0;
    float** phase = nullptr;

    //mixer attributes
    int winsize = 48;
    float fc = 5e3;
    trianglut lut;
    int lut_idx;
};


// digital up converter runnable
struct ddc:runnable{
public:
    ddc(scheduler *sch,  pipebuf<float> &_in, pipebuf_c<float> &_out, const link_cfg& _cfg)
    :runnable(sch, "ddc"),in(_in), out(_out), cfg(_cfg),
    lut(fc, _cfg.sr, 48u << static_cast<unsigned>(_cfg.bw)){
        up = 1u << static_cast<unsigned>(cfg.bw); // 1, 2, 4, 8
        down = 25;
        filter_design();
        lut_window = 48 * up;
        lut_idx =0;
    }
    void filter_design(){
        // filter design
        delete[] re_filt;
        delete[] im_filt;
        const float cutoff = 0.5f / down;
        const float transition = (1.0f - cfg.bandwidth / cfg.sr) / down;
        // apply design and obtain taps
        Kaiserparam p = estimateKaiser(60, transition);
        L_phase = (p.numTaps + up - 1) / up;
        float* h = new float[up*L_phase]{};
        kaiserLpf(h, cutoff, p);
        scale(h, (float)up, p.numTaps);
        re_filt = new float[up*L_phase]{};
        im_filt = new float[up*L_phase]{};
        for(int i = 0; i < up; i++){
            for(unsigned int j = 0; j < L_phase; ++j){
                re_filt[L_phase * (i + 1) - 1 - j] = 2*h[i + up * j]
                * std::cos(2 * pi * fc * (i + up * j)/(48e3*up));
                im_filt[L_phase * (i + 1) - 1 - j] = 2*h[i + up * j]
                * std::sin(2 * pi * fc * (i + up * j)/(48e3*up));     

            }
        }
        delete[] h;
    }
    void run() override {
        const float* rd = in.rd();
        const size_t available = in.readable();
        const size_t capacity = out.writable();
        cv32 wr = out.wr_c();

        const size_t required = std::max(
            size_t(L_phase), size_t((down + up - 1) / up));

        size_t consumed = 0;
        size_t produced = 0;

        while (available - consumed >= required && produced < capacity) {
            const float* re_ph = re_filt + filter_phase * L_phase;
            const float* im_ph = im_filt + filter_phase * L_phase;
            const float* input = rd + consumed;
            float re = 0.0f;
            float im = 0.0f;
            for (unsigned j = 0; j < L_phase; ++j) {
                const float re_h = re_ph[j];
                const float im_h = im_ph[j];
                re += input[j] * re_h;
                im += input[j] * im_h;
            }
            const float c = lut.cos[lut_idx];
            const float s = lut.sin[lut_idx];
            wr.re[produced] = re * c + im * s;
            wr.im[produced++] = im * c - re * s;
            filter_phase += down;
            consumed += filter_phase / up;
            filter_phase %= up;
            if (++lut_idx == lut_window)
                lut_idx = 0;
        }
        in.read(consumed);
        out.written(produced);
    }
    void set_fc(int fc_kHz) {
        if (fc_kHz < 2 || fc_kHz > 18)
            fail("wrong centre frequency was attempted");

        fc = fc_kHz * 1000.0f;
        filter_design();
        lut.update(fc, cfg.sr);
        lut_window = 48 * up;
}
    ~ddc(){
        delete[] re_filt; delete[] im_filt;
    }
private:
    pipereader<float> in;
    pipewriter <float> out;

    // resampler attributes
    link_cfg cfg;
    int up;
    int down;
    unsigned L_phase = 0;
    unsigned filter_phase = 0;
    float *re_filt = nullptr, *im_filt = nullptr;

    //mixer attributes
    float fc = 5e3;
    int lut_window;
    trianglut lut;
    int lut_idx;
};

// baseband signal generator
struct ofdm_modulator: runnable{
    ofdm_modulator(scheduler *sch,  pipebuf_c<float> &_in, pipebuf_c<float> &_out, 
    const link_cfg& _cfg): runnable(sch, "ofdm_modulator"),cfg(_cfg),
    in(_in), out(_out, (cfg.nFFT + cfg.cp) * cfg.nSym){
        fft_engines[0] = new fft<128>();fft_engines[1] = new fft<256>();
        fft_engines[2] = new fft<512>();fft_engines[3] = new fft<1024>();
        
    }
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
        size_t fft_index = static_cast<size_t>(cfg.bw);
        size_t sym_index = 0;
        while(sym_index < cfg.nSym){
            for(size_t i = 0; i < cfg.nSC/2; ++i){
                symout[out_offset + i] = symin[i];
                symout[cfg.cp + 1 + i] = symin[in_offset + i];    
            }
            fft_engines[fft_index]->inverse(symout + cfg.cp);
            std::memmove(symout.re, symout.re + cfg.nFFT, sizeof(float) * cfg.cp);
            std::memmove(symout.im, symout.im + cfg.nFFT, sizeof(float) * cfg.cp);
            symin += cfg.nSC;
            symout += cfg.nFFT + cfg.cp;
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
    pipereader<float> in;
    pipewriter <float> out;


};
struct timesync{
    static constexpr size_t window = 4096;
    timesync(const link_cfg& cfg, const fft_base& symbol_fft):
    pss{new float[window]{}, new float[window]{}},
    corr{new float[window]{}, new float[window]{}}{
        constexpr int root = 25;
        for (int n = 0; n < 62; ++n) {
            int k = (n < 31) ? n : n + 1;
            int c = (n < 31) ? (cfg.nFFT - 31 + n):(n - 30);
            float phase = -pi * root * k * (k + 1) / 63.0;
            pss.re[c] = std::cos(phase);
            pss.im[c] = std::sin(phase);
        }
        symbol_fft.inverse(pss);
        fft_engine.forward(pss);

        pssEnergy = 0;
        for (size_t i = 0; i < window; ++i){
            pss.im[i] = -pss.im[i];
            pssEnergy += std::pow(pss.im[i], 2) + std::pow(pss.re[i], 2);
        }
        npss = cfg.nFFT;
        pssEnergy /= window;
        th = 62.0f / (cfg.nSC - 10.0f + cfg.nFFT); // empirical formula
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
        int peak_idx = 0;
        for (size_t i = 0; i < valid; ++i){
            float c = abs(corr[i]);
            c /= (sigEnergy * pssEnergy + 1e-7);
            if (i + 1 < valid)
                sigEnergy += abs(sig[i + npss]) - abs(sig[i]);
            if (c > peak){
                peak = c;
                peak_idx = i;
            }
        }
        if (peak > th) return peak_idx;
        return -1;
    }
private:
    float th;
    fft<window> fft_engine;
    cv32 pss;
    cv32 corr;
    size_t npss;
    float pssEnergy;
};
struct freqsync{
    freqsync(const link_cfg& cfg):
    osc(1), fr(0),nfft(cfg.nFFT), cp(cfg.cp), sr(cfg.sr){
        timer = 0;
        step = 1;
    };
    void freq_est(cv32 ofdm){
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
        if (fr == 0) fr = _fr;
        fr = 0.8 * fr + 0.2 * _fr;
        float angle = -2 * pi * fr / sr;
        step = cf32(std::cos(angle), std::sin(angle));
        osc /= std::abs(osc);
    }
    void sync(cv32 ofdm){
        if (timer++ == 0) freq_est(ofdm);
        timer &= 0x03;
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
    float fr;
    float sr;
    int nfft, cp;
    int timer = 0;
};
struct ofdm_demodulator: runnable{
    ofdm_demodulator(scheduler *sch, pipebuf_c<float> &_in,
    pipebuf_c<float> &_out, const link_cfg& _cfg):
    runnable(sch, "ofdm_demodulator"),
    cfg(_cfg), in(_in), out(_out, _cfg.nSC * _cfg.nSym){
        fft_engines[0] = new fft<128>();fft_engines[1] = new fft<256>();
        fft_engines[2] = new fft<512>();fft_engines[3] = new fft<1024>();

        buffer_re = new float[cfg.nFFT + cfg.cp];
        buffer_im = new float[cfg.nFFT + cfg.cp];
        size_t fft_index = static_cast<size_t>(cfg.bw);
        tsync = new timesync(_cfg, *fft_engines[fft_index]);
        fsync = new freqsync(_cfg);
    }

    void run() override{
        while (!sync) {
            if (in.readable() < cfg.cp + cfg.nFFT)
                return;
            int searched = std::min(in.readable() - cfg.cp, tsync->window);
            int t = tsync->sync(in.rd_c() + cfg.cp, searched);
            if (t == -1) {
                in.read(searched - cfg.nFFT + 1);
                continue;
            }
            in.read((size_t)t);
            sync = true;
        }
        if(in.readable() < (cfg.nFFT + cfg.cp) * cfg.nSym ||
        out.writable() < cfg.nSC * cfg.nSym)
            return;

        cv32 symin = in.rd_c();
        cv32 symout = out.wr_c();


        size_t in_offset = cfg.nFFT + cfg.cp - cfg.nSC/2;
        size_t out_offset = cfg.nSC/2;
        size_t fft_index = static_cast<size_t>(cfg.bw);
        size_t sym_index = 0;

        while(sym_index < cfg.nSym){
            std::memcpy(buffer_re, symin.re, sizeof(float) * (cfg.cp+cfg.nFFT));
            std::memcpy(buffer_im, symin.im, sizeof(float) * (cfg.cp+cfg.nFFT));

            fsync->sync(cv32{buffer_re, buffer_im});
            fft_engines[fft_index]->forward(cv32{buffer_re + cfg.cp, buffer_im + cfg.cp});

            for(size_t i = 0; i < cfg.nSC/2; ++i){
                symout.re[i] = buffer_re[in_offset + i];
                symout.im[i] = buffer_im[in_offset + i];
                symout.re[out_offset + i] = buffer_re[cfg.cp + 1 + i];
                symout.im[out_offset + i] = buffer_im[cfg.cp + 1 + i];
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
        delete[] buffer_re;
        delete[] buffer_im;
        delete tsync;
        delete fsync;
    }

private:
    const link_cfg& cfg;
    fft_base* fft_engines[4];
    pipereader<float> in;
    pipewriter<float> out;
    float* buffer_re;
    float* buffer_im;
    timesync* tsync;
    freqsync* fsync;
    bool sync = false;
};
}


#endif
