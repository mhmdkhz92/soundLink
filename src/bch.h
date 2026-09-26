#ifndef SOUNDLINK_BCH_H
#define SOUNDLINK_BCH_H

#include <cstdint>
#include <cstddef>
#include <cassert>


template <int M, uint16_t Polynomial>
    struct GF{
    public:
        static constexpr int SIZE  = 1 << M;
        static constexpr int ORDER = SIZE - 1;
    private:
        uint16_t value;

        static inline uint16_t exp_table[ORDER];
        static inline int16_t  log_table[SIZE];

        static inline bool initialized = false;

    public:
        GF(uint16_t v = 0):value(v){
            if (!initialized)
                init();
        }
        static void init(){
            for (int i = 0; i < SIZE; ++i)
                log_table[i] = -1;
            uint16_t x = 1;
            for (int i = 0; i < ORDER; ++i){
                exp_table[i] = x;
                log_table[x] = i;
                bool overflow = (x & (1u << (M - 1))) != 0;
                x <<= 1;
                if (overflow)
                    x ^= Polynomial;
                x &= ORDER;
            }
            // alpha must have order 2^M - 1
            assert(x == 1);
            initialized = true;
        }

        GF operator+(const GF& other) const{
            return GF(value ^ other.value);
        }

        GF operator-(const GF& other) const{
            return GF(value ^ other.value);
        }

        GF operator*(const GF& other) const{
            if (value == 0 || other.value == 0)
                return GF(0);
            int exponent = log_table[value] + log_table[other.value];
            if (exponent >= ORDER)
                exponent -= ORDER;
            return GF(exp_table[exponent]);
        }

        GF operator/(const GF& other) const{
            assert(other.value != 0);
            if (value == 0)
                return GF(0);
            int exponent = log_table[value] - log_table[other.value];
            if (exponent < 0)
                exponent += ORDER;
            return GF(exp_table[exponent]);
        }

        static GF alpha(int exponent){
            if (!initialized)
                init();
            exponent %= ORDER;
            if (exponent < 0)
                exponent += ORDER;
            return GF(exp_table[exponent]);
        }

        bool operator==(const GF& other) const{
            return value == other.value;
        }

        bool operator!=(const GF& other) const{
            return value != other.value;
        }

        uint16_t raw() const{
            return value;
        }
    };

template <typename gf>
    struct poly{
        static constexpr int MAX_TERMS = 8;
        gf c[MAX_TERMS]{};
        int degree = 0;

        gf& operator[](int i){
            return c[i];
        }

        void clear(){
            for (int i = 0; i < MAX_TERMS; ++i)
                c[i] = gf(0);
            degree = 0;
        }
    };

template <typename gf>
bool berlekamp_massey(gf* syndromes, int syndrome_count, poly<gf>& lambda)
{
    // BM working polynomials
    poly<gf> B;
    poly<gf> T;

    lambda.clear();
    B.clear();

    lambda[0] = gf(1);
    B[0]      = gf(1);

    int L = 0;
    int m = 1;

    gf b = gf(1);

    for (int n = 0; n < syndrome_count; ++n){
        gf d = syndromes[n];
        for (int i = 1; i <= L; ++i){
            d = d + lambda[i] * syndromes[n - i];
        }

        if (d == gf(0)){
            ++m;
            continue;
        }
        T = lambda;
        gf factor = d / b;
        for (int i = 0; i <= B.degree; ++i){
            int target = i + m;
            // Our polynomial container has finite capacity.
            if (target >= poly<gf>::MAX_TERMS)
                return false;

            lambda[target] = lambda[target] + factor * B[i];
        }

        if (2 * L <= n){
            int new_L = n + 1 - L;
            B = T;
            b = d;
            L = new_L;
            m = 1;
        }
        else{
            ++m;
        }
        lambda.degree = L;
        if (L >= poly<gf>::MAX_TERMS)
            return false;
    }
    lambda.degree = L;
    return true;
}

template <typename gf>
bool chien_search(poly<gf>& lambda, int code_length, int* error_positions, int& error_count){
    error_count = 0;
    if (lambda.degree == 0)
        return true;

    gf R[poly<gf>::MAX_TERMS];
    for (int j = 1; j <= lambda.degree; ++j)
        R[j] = lambda[j];

    gf step[poly<gf>::MAX_TERMS];

    for (int j = 1; j <= lambda.degree; ++j)
        step[j] = gf::alpha(-j);

    for (int i = 0; i < code_length; ++i){
        gf value = gf(1);
        for (int j = 1; j <= lambda.degree; ++j)
            value = value + R[j];

        if (value == gf(0)){
            if (error_count >= lambda.degree)
                return false;
            error_positions[error_count] = i;
            ++error_count;
        }
        for (int j = 1; j <= lambda.degree; ++j)
            R[j] = R[j] * step[j];
    }
    return error_count == lambda.degree;
}

struct bch63_51{
    static constexpr int N = 63;
    static constexpr int K = 51;
    static constexpr int T = 2;
    static constexpr int PARITY_BITS = N - K;
    using gf = GF<6, 0x03>;
    // --------------------------------------------------------
    // Generator polynomial
    // g(x) = x^12 + x^10 + x^8 + x^5 + x^4 + x^3 + 1
    // binary: 1010100111001
    // hex:    0x1539
    // --------------------------------------------------------
    static constexpr uint64_t GENERATOR = 0x1539;

    uint64_t encode(uint64_t message){
        // Keep only the lower 51 message bits.
        message &= (1ULL << K) - 1;
        // Multiply m(x) by x^12.
        uint64_t codeword = message << PARITY_BITS;

        // Polynomial division working copy.
        uint64_t remainder = codeword;

        // Divide by g(x).
        for (int i = N - 1; i >= PARITY_BITS; --i){
            if (remainder & (1ULL << i)){
                remainder ^= GENERATOR << (i - PARITY_BITS);
            }
        }
        // Only the lower 12 bits are the remainder.
        remainder &= (1ULL << PARITY_BITS) - 1;
        return codeword | remainder;
    }
    gf syndrome(uint64_t received, int j){
    gf s(0);
    for (int i = 0; i < N; ++i){
        if (received & (1ULL << i))
            s = s + gf::alpha(i * j);
    }
    return s;
    }

    void syndromes(uint64_t received, gf* S){
        S[0] = gf(0);   // S1
        S[2] = gf(0);   // S3

        for (int i = 0; i < N; ++i){
            if (received & (1ULL << i)){
                S[0] = S[0] + gf::alpha(i);
                S[2] = S[2] + gf::alpha(3 * i);
            }
        }
        // Frobenius relations for binary BCH codes
        S[1] = S[0] * S[0];   // S2
        S[3] = S[1] * S[1];   // S4
    }
    bool decode(uint64_t& received){
        gf S[2 * T]; 
        syndromes(received, S);
        bool clean = true;
        for (int i = 0; i < 2 * T; ++i){
            if (S[i] != gf(0)){
                clean = false;
                break;
            }
        }
        if (clean)
            return true;

        poly<gf> lambda;
        if (!berlekamp_massey(S, 2 * T, lambda)){
        return false;
        }

        // This particular BCH code can correct at most 2 errors.
        if (lambda.degree > T)
            return false;


        int error_positions[poly<gf>::MAX_TERMS];
        int error_count = 0;
        if (!chien_search(lambda, N, error_positions, error_count)){
            return false;
        }
        for (int i = 0; i < error_count; ++i){
            received ^= 1ULL << error_positions[i];
        }
        gf check[2 * T];

        syndromes(received, check);

        for (int i = 0; i < 2 * T; ++i)
        {
            if (check[i] != gf(0))
                return false;
        }
        return true;
    }
};

#endif