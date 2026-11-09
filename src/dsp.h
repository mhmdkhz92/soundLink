#ifndef SOUNDLINK_DSP_H
#define SOUNDLINK_DSP_H

#include "framework.h"
#include "math.h"
#include "filter.h"
#include <algorithm>
#include <cmath>

namespace soundlink{

// digital up converter runnable
struct duc:runnable{
public:
    duc(scheduler *sch,  pipebuf_c<float> &_in, pipebuf<float> &_out, const link_cfg& cfg)
    :runnable(sch, "duc"),
    in(_in), out(_out), lut_idx(0), lut(5e3, 48e3, winsize){
        if (cfg.fc < 2 || cfg.fc > 18)
            fail("wrong centre frequency was attempted");

        float fc = static_cast<float>(cfg.fc) * 1000.0f;
        lut.update(fc, 48000.0f);

        up = 25;
        down = 1u << static_cast<unsigned>(cfg.bw); // 1, 2, 4, 8
        // filter design
        const float cutoff = 0.5f / static_cast<float>(up);
        const float transition = (1.0f - cfg.bandwidth / cfg.sr) / static_cast<float>(up);

        // apply design and obtain taps
        Kaiserparam p = estimateKaiser(60, transition);
        phaseLen = (p.numTaps + up - 1) / up;

        float* h = new float[up*phaseLen]{};
        filter = new float[up*phaseLen]{};
        kaiserLpf(h, cutoff, p);
        scale(h, (float)up, p.numTaps);


        phase = new float*[up];
        for(size_t i = 0; i < up; i++){
            phase[i] = filter + (i* phaseLen);
            for(size_t j = 0; j < phaseLen; ++j){
                filter[phaseLen * (i + 1) - 1 - j] = h[i + up * j];
            }
        }
        delete[] h;
    }
    void run() override {
        const cv32 rd = in.rd_c();
        const size_t available = in.readable();
        const size_t capacity = out.writable();
        float* wr = out.wr();

        const size_t required = std::max(phaseLen, (down + up - 1) / up);

        size_t consumed = 0;
        size_t produced = 0;

        while (available - consumed >= required && produced < capacity) {
            const float* ph = phase[filter_phase];
            const float* input_re = rd.re + consumed;
            const float* input_im = rd.im + consumed;
            float re = 0.0f;
            float im = 0.0f;
            for (size_t j = 0; j < phaseLen; ++j) {
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
    ~duc(){
        delete[] phase;
        delete[] filter;
    }
private:
    pipereader<float> in;
    pipewriter <float> out;

    // resampler attributes
    size_t up;
    size_t down;
    size_t phaseLen = 0;
    size_t filter_phase = 0;
    float** phase = nullptr;
    float* filter = nullptr;

    //mixer attributes
    static constexpr size_t winsize = 48;
    size_t lut_idx;
    trianglut lut;
};


// digital up converter runnable
struct ddc:runnable{
public:
    ddc(scheduler *sch,  pipebuf<float> &_in, pipebuf_c<float> &_out, const link_cfg& _cfg)
    :runnable(sch, "ddc"),in(_in), out(_out), cfg(_cfg),
    lut(cfg.fc * 1000.0f, cfg.sr, 48u << (unsigned)cfg.bw){
        float fc = cfg.fc * 1000.0f;
        up = 1u << static_cast<unsigned>(cfg.bw); // 1, 2, 4, 8
        down = 25;
    
        const float cutoff = 0.5f / static_cast<float>(down);
        const float transition = (1.0f - cfg.bandwidth / cfg.sr) / static_cast<float>(down);
        // apply design and obtain taps
        Kaiserparam p = estimateKaiser(60, transition);
        phaseLen = (p.numTaps + up - 1) / up;
        float* h = new float[up*phaseLen]{};
        kaiserLpf(h, cutoff, p);
        scale(h, (float)up, p.numTaps);
        re_filt = new float[up*phaseLen]{};
        im_filt = new float[up*phaseLen]{};
        for(size_t i = 0; i < up; i++){
            for(size_t j = 0; j < phaseLen; ++j){
                float angle = 2.0f * pi * fc * (float)(i + up * j) / (48000.0f * (float)up);
                re_filt[phaseLen * (i + 1) - 1 - j] = 2*h[i + up * j] * std::cos(angle);
                im_filt[phaseLen * (i + 1) - 1 - j] = 2*h[i + up * j] * std::sin(angle);     
            }
        }
        lut_window = 48 * up;
        lut_idx =0;
        delete[] h;
    }
    
    void run() override {
        const float* rd = in.rd();
        const size_t available = in.readable();
        const size_t capacity = out.writable();
        cv32 wr = out.wr_c();

        const size_t required = std::max(phaseLen, (down + up - 1) / up);

        size_t consumed = 0;
        size_t produced = 0;

        while (available - consumed >= required && produced < capacity) {
            const float* re_ph = re_filt + filter_phase * phaseLen;
            const float* im_ph = im_filt + filter_phase * phaseLen;
            const float* input = rd + consumed;
            float re = 0.0f;
            float im = 0.0f;
            for (size_t j = 0; j < phaseLen; ++j) {
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
    ~ddc(){
        delete[] re_filt; delete[] im_filt;
    }
private:
    pipereader<float> in;
    pipewriter <float> out;

    // resampler attributes
    link_cfg cfg;
    size_t up;
    size_t down;
    size_t phaseLen = 0;
    size_t filter_phase = 0;
    float *re_filt = nullptr, *im_filt = nullptr;

    //mixer attributes
    size_t lut_window;
    trianglut lut;
    size_t lut_idx;
};
}


#endif
