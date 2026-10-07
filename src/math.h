#ifndef SOUNDLINK_MATH_H
#define SOUNDLINK_MATH_H

#include <math.h>
#include <complex>
#include <climits>
#include <type_traits>
#include "framework.h"

namespace soundlink{
constexpr float pi = 3.14159265358979323846f;
// scalar
template<typename T>
inline void scale(T* a, T scalar, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        a[i] *= scalar;
    }
}

float dot_product(const float* __restrict a,
    const float* __restrict b, unsigned N) noexcept {
    if (N > INT_MAX)
        fail("dot_product count exceeds INT_MAX");
    float result = 0;
    for (unsigned i = 0; i < N; ++i)
        result += a[i] * b[i];
    return result;
}

void product_c(const cv32 __restrict a,
    const cv32 __restrict b, cv32 __restrict c, unsigned N){
        for(unsigned i = 0; i < N; ++i){
            c.re[i] = a.re[i] * b.re[i] - a.im[i]*b.im[i];
            c.im[i] = a.re[i] * b.im[i] + a.im[i]*b.re[i];
        }
}
void product_c(cv32 __restrict a,
    const cv32 __restrict b, unsigned N){
        float re, im;
        for(unsigned i = 0; i < N; ++i){
            re = a.re[i] * b.re[i] - a.im[i]*b.im[i];
            im = a.re[i] * b.im[i] + a.im[i]*b.re[i];
            a.re[i] = re; a.im[i] = im;
        }
}

template<typename T>
inline T min(const T* a, size_t size) {
    T res = a[0];

    for (size_t i = 1; i < size; ++i) {
        if (a[i] < res) {
            res = a[i];
        }
    }
    return res;
}

template<typename T>
inline T max(const T* a, size_t size) {
    T res = a[0];

    for (size_t i = 1; i < size; ++i) {
        if (a[i] > res) {
            res = a[i];
        }
    }
    return res;
}
struct trianglut {
    float* sin;
    float* cos;
    size_t size;

    trianglut(float Fc, float Fs, size_t N)
        : sin(new float[N]), cos(new float[N]), size(N) {
        update(Fc, Fs);
    }

    void update(float Fc, float Fs) {
        for (size_t i = 0; i < size; ++i) {
            const float phase = 2.0f * pi * Fc * (float)i / Fs;
            sin[i] = std::sin(phase);
            cos[i] = std::cos(phase);
        }
    }
    ~trianglut() {
        delete[] sin;
        delete[] cos;
    }
};


struct fft {
private:
    const size_t N;
    size_t* bit_rev;
    float* tw_re;
    float* tw_im;

    template<bool Inverse>
    void stages(float* __restrict re, float* __restrict im) const {
        for (size_t Step = 1; Step < N; Step *= 2) {
            const float* tr = tw_re + Step - 1;
            const float* ti = tw_im + Step - 1;
            for (size_t i = 0; i < N; i += 2 * Step) {
                for (size_t j = 0; j < Step; ++j) {
                    const size_t a = i + j;
                    const size_t b = a + Step;
                    const float wr = tr[j];
                    const float wi = Inverse ? -ti[j] : ti[j];
                    const float xr = wr * re[b] - wi * im[b];
                    const float xi = wr * im[b] + wi * re[b];
                    const float ur = re[a];
                    const float ui = im[a];
                    re[a] = ur + xr;
                    im[a] = ui + xi;
                    re[b] = ur - xr;
                    im[b] = ui - xi;
                }
            }
        }
    }
    template<bool Inverse>
    void process(cv32 data) const {
        float* re = data.re;
        float* im = data.im;
        for (size_t i = 0; i < N; ++i) {
            const size_t j = bit_rev[i];
            if (i < j) {
                const float r = re[i];
                const float v = im[i];
                re[i] = re[j];
                im[i] = im[j];
                re[j] = r;
                im[j] = v;
            }
        }
        stages<Inverse>(re, im);
        if (Inverse) {
            const float invN = 1.0f / (float)N;
            for (size_t i = 0; i < N; ++i) {
                re[i] *= invN;
                im[i] *= invN;
            }
        }
    }
public:
    explicit fft(size_t N): N(N), bit_rev(nullptr), tw_re(nullptr), tw_im(nullptr) {
        if (N < 2 || (N & (N - 1)) != 0)
            fail("FFT length must be a power of two >= 2");
        try {
            bit_rev = new size_t[N];
            tw_re = new float[N - 1];
            tw_im = new float[N - 1];
        } catch (...) {
            delete[] bit_rev;
            delete[] tw_re;
            delete[] tw_im;
            throw;
        }
        for (size_t i = 0; i < N; ++i) {
            size_t rev = 0;
            for (size_t n = N, bits = i; n > 1; n >>= 1, bits >>= 1)
                rev = (rev << 1) | (bits & 1);
            bit_rev[i] = rev;
        }
        for (size_t step = 1; step < N; step *= 2) {
            for (size_t j = 0; j < step; ++j) {
                const double angle = -2.0 * 3.14159265358979323846 * (double)j / (2.0 * (double)step);
                tw_re[step - 1 + j] = float(std::cos(angle));
                tw_im[step - 1 + j] = float(std::sin(angle));
            }
        }
    }

    ~fft() {
        delete[] bit_rev;
        delete[] tw_re;
        delete[] tw_im;
    }

    fft(const fft&) = delete;
    fft& operator=(const fft&) = delete;

    void forward(cv32 data) const {
        process<false>(data);
    }

    void inverse(cv32 data) const {
        process<true>(data);
    }
};
}
#endif
