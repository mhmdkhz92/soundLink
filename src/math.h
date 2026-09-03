#ifndef SOUNDLINK_MATH_H
#define SOUNDLINK_MATH_H

#include <math.h>
#include <complex>


constexpr float pi = 3.1415926535f;
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

    __declspec(noinline)
    float dot_product(const float* __restrict a,
        const float* __restrict b, int count) noexcept {
        float result = 0.0f;
        for (int i = 0; i < count; ++i)
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
        for (int i = 0; i < N; ++i){
            term[i] = 1;
            y[i] = 1;
            x2[i] = x[i] * x[i] / 4;
        }
        while (1){
            for (int i = 0; i < N; ++i){
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
}
#endif

