#ifndef SOUNDLINK_MATH_H
#define SOUNDLINK_MATH_H

#include <math.h>
#include <complex>
#include "framework.h"


constexpr float pi = 3.14159265358979323846f;
namespace soundlink {
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
        float result = 0.0f;
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
    struct trianglut{
        float* sin;
        float* cos;
        trianglut(float Fc, float Fs, int N){
            sin = new float[N];
            cos = new float[N];
            float t = 0;
            for (int i = 0; i < N; ++i){
                sin[i] = std::sinf(2*pi*Fc * t);
                cos[i] = std::cosf(2*pi*Fc * t);
                t += 1/Fs;
            }
        }
        ~trianglut(){
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
    virtual void forward(cf32* data) const = 0;
    virtual void inverse(cf32* data) const = 0;
    };

    template <size_t N>
    struct fft:fft_base {
        static_assert(N == 128 || N == 256 || N == 512 || N == 1024, "wrong length");
    private:
        size_t bit_rev[N];
        cf32 twiddles[N / 2];

        void process(cf32* data, bool inverse) const {

            // 1. Bit-reversal permutation
            for (size_t i = 0; i < N; i++) {
                size_t rev = bit_rev[i];
                if (i < rev) {
                    cf32 temp = data[i];
                    data[i] = data[rev];
                    data[rev] = temp;
                }
            }

            // 2. Cooley-Tukey Radix-2 DIT computation
            for (size_t step = 1; step < N; step *= 2) {
                size_t jump = step * 2;
                size_t twiddle_step = (N / 2) / step;

                for (size_t i = 0; i < N; i += jump) {
                    // inner loop
                    for (size_t j = 0; j < step; j++) {
                        cf32 twiddle = twiddles[j * twiddle_step];
                        if (inverse) {
                            twiddle = std::conj(twiddle);
                        }

                        cf32 t = twiddle * data[i + j + step];
                        cf32 u = data[i + j];
                        
                        data[i + j] = u + t;
                        data[i + j + step] = u - t;
                    }
                }
            }

            // 3. Normalization for IFFT
            if (inverse) {
                const float invN = 1.0f / N;
                for (size_t i = 0; i < N; i++) {
                    data[i] *= invN;
                }
            }
        }

    public:
        fft() {
            int log2N = 0;
            while ((1ULL << log2N) < N) log2N++;

            for (size_t i = 0; i < N; i++) {
                size_t rev = 0;
                for (int j = 0; j < log2N; j++) {
                    if ((i >> j) & 1) {
                        rev |= (1ULL << (log2N - 1 - j));
                    }
                }
                bit_rev[i] = rev;
            }

            for (size_t i = 0; i < N / 2; i++) {
                float angle = -2.0f * pi * i / N;
                twiddles[i] = cf32(std::cosf(angle), std::sinf(angle));
            }
        }

        void forward(cf32* data) const override {
            process(data, false);
        }

        void inverse(cf32* data) const override {
            process(data, true);
        }
    };
}
#endif

