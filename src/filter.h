#ifndef SOUNDLINK_FILTER_H
#define SOUNDLINK_FILTER_H

#include "math.h"

namespace soundlink{

   struct Kaiserparam{
        float beta;
        size_t numTaps;
    };

    void besselI0(float *x, float *y, size_t N){
        float *term = new float[N];
        float *x2 = new float[N];
        float k = 1;
        for (size_t i = 0; i < N; ++i){
            term[i] = 1;
            y[i] = 1;
            x2[i] = x[i] * x[i] / 4;
        }
        while (1){
            for (size_t i = 0; i < N; ++i){
                term[i] *= x2[i] / (k * k);
                y[i] += term[i];
            }
            k++;
            if (max(term, N) <= max(y, N) * 1e-15)
                break;
        }
        delete[] term;
        delete[] x2;
    }

    inline void kaiserWin(float *y, Kaiserparam p){
        float x = 0;
        float *arg = new float[p.numTaps];
        for (size_t i = 0; i < p.numTaps; ++i){
            const float position = 2.0f * x / (float)(p.numTaps - 1) - 1.0f;
            float t = position * position;
            arg[i] = p.beta * std::sqrt(1 - t);
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
                ideal = std::sin(2.0f * pi * cutoff * (float)m)/ (pi * (float)m);
            coefficients[n] *= ideal;
            sum += coefficients[n];
        }

        // Unity gain at DC
        scale(coefficients, 1/sum, p.numTaps);
    }

}

#endif