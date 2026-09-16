#ifndef SOUNDLINK_MATH_H
#define SOUNDLINK_MATH_H

#include <math.h>
#include <complex>
#include <climits>
#include <type_traits>
#include "framework.h"


constexpr float pi = 3.14159265358979323846f;
namespace soundlink{
    //multiply
    
    template<typename T>
    inline void multiply(const T* a, const T* b, T* c, size_t size){
        for (size_t i = 0; i < size; ++i){
            c[i] = a[i] * b[i];
        }
    }

    //add
    template<typename T>
    inline void add(const T* a, const T* b, T* c, size_t size){
        for (size_t i = 0; i < size; ++i){
            c[i] = a[i] + b[i];
        }
    }

    // power
    template<typename T>
    inline void power(const T* a, T* b, T exponent, size_t size) {
        for (size_t i = 0; i < size; ++i) {
            b[i] = std::pow(a[i], exponent);
        }
    }

    // In-place multiply
    template<typename T>
    inline void multiply(T* a, const T* b, size_t size) {
        multiply(a, b, a, size);
    }

    // In-place add
    template<typename T>
    inline void add(T* a, const T* b, size_t size) {
        add(a, b, a, size);
    }

    // In-place power
    template<typename T>
    inline void power(T* a, T exponent, size_t size) {
        power(a, a, exponent, size);
    }

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

    // create an array of floating points from 0 to N-1
    template<typename T>
    inline void arange(T* a, T start, T step, size_t N){
        T t = start;
        for (size_t i = 0; i < N; ++i) {
            a[i] = (T)t;
            t += step;
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
                const float phase = 2 * pi * Fc * i / Fs;
                sin[i] = std::sin(phase);
                cos[i] = std::cos(phase);
            }
        }
        ~trianglut() {
            delete[] sin;
            delete[] cos;
        }
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

    struct fft_base {
        virtual ~fft_base() = default;
        virtual void forward(cv32 data) const = 0;
        virtual void inverse(cv32 data) const = 0;
    };

    template<size_t N>
    struct fft:fft_base {
        static_assert(N == 128 || N == 256 || N == 512 || N == 1024, "wrong length");
    private:
        size_t bit_rev[N];
        alignas(64) float tw_re[N - 1];
        alignas(64) float tw_im[N - 1];

        template<size_t Step, bool Inverse>
        typename std::enable_if<Step == N>::type stages(float*, float*) const {}

        template<size_t Step, bool Inverse>
        typename std::enable_if<(Step < N)>::type stages(
            float* __restrict re, float* __restrict im) const {
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
            stages<Step * 2, Inverse>(re, im);
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
            stages<1, Inverse>(re, im);
            if (Inverse) {
                for (size_t i = 0; i < N; ++i) {
                    re[i] *= 1.0f / N;
                    im[i] *= 1.0f / N;
                }
            }
        }
    public:
        fft(){
            for (size_t i = 0; i < N; ++i) {
                size_t rev = 0;
                for (size_t n = N, bits = i; n > 1; n >>= 1, bits >>= 1)
                    rev = (rev << 1) | (bits & 1);
                bit_rev[i] = rev;
            }
            for (size_t step = 1; step < N; step *= 2) {
                for (size_t j = 0; j < step; ++j) {
                    const double angle = -6.28318530717958647692 * j / (2 * step);
                    tw_re[step - 1 + j] = float(std::cos(angle));
                    tw_im[step - 1 + j] = float(std::sin(angle));
                }
            }
        }

        void forward(cv32 data) const override {
            process<false>(data);
        }

        void inverse(cv32 data) const override {
            process<true>(data);
        }
    };
}
#endif
