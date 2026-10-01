#ifndef SOUNDLINK_BCH_H
#define SOUNDLINK_BCH_H

#include <cstdint>
#include <cstddef>
#include <cassert>

#include "framework.h"

namespace soundlink{
struct bch_config {
    int M, N, K, T;  // GF(2^M)
    uint16_t primitive; 
    uint64_t generator;
    constexpr int msg_bytes() const {
        return (K + 7) >> 3;
    }
    constexpr int code_bytes() const {
        return (N + 7) >> 3;
    }
    constexpr int parity_bits() const {
        return N - K;
    }
};
static constexpr bch_config bch63_51{
    6, 63, 51, 2,
    0x03,       // primitive
    0x1539      // generator
};

static constexpr bch_config bch127_106{
    7, 127, 106, 3,
    0x09,       // primitive
    0x26D9E3    // generator
};

template <int M, uint16_t Polynomial>
struct GF{
public:
    static const int SIZE  = 1 << M;
    static const int ORDER = SIZE - 1;
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
    static const int MAX_TERMS = 8;
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

inline void bch_encode(uint8_t * code, const bch_config& cfg) {
    const int parity_bits = cfg.parity_bits();
    const uint64_t mask = UINT64_MAX >> (64 - parity_bits);

    uint64_t remainder = 0;


    for (int i = 0; i < cfg.K; ++i) {
        uint64_t bit = (code[i >> 3] >> (7 - (i & 7))) & 1;

        uint64_t feedback = ((remainder >> (parity_bits - 1)) & 1) ^ bit;

        remainder = (remainder << 1) & mask;

        if (feedback)
            remainder ^= cfg.generator & mask;
    }

    for (int i = parity_bits - 1, j = cfg.K; i >= 0; --i, ++j) {
        const unsigned shift = 7 - (j & 7);
        const unsigned bit = (remainder >> i) & 1;
        code[j >> 3] = (code[j >> 3] & ~(1u << shift)) | (bit << shift);
    }
}

template<typename gf>
void syndromes(const uint8_t* received, gf* S, const bch_config& config) {
    // Initialize odd syndromes.
    for (int j = 1; j < 2 * config.T; j += 2)
        S[j - 1] = gf(0);

    // Read the codeword MSB-first.
    for (int i = 0, power = config.N - 1; i < config.N; ++i, --power) {
        bool bit = received[i / 8] & (0x80u >> (i % 8));

        if (bit) {
            for (int j = 1; j < 2 * config.T; j += 2)
                S[j - 1] = S[j - 1] + gf::alpha(power * j);
        }
    }
    // Derive even syndromes by squaring.
    for (int j = 2; j <= 2 * config.T; j += 2)
        S[j - 1] = S[j / 2 - 1] * S[j / 2 - 1];
}

template<typename gf>
bool bch_decode(uint8_t* received, const bch_config& config) {
    const int syndrome_count = 2 * config.T;

    gf S[2 * (poly<gf>::MAX_TERMS - 1)];
    syndromes(received, S, config);
    bool clean = true;
    for (int i = 0; i < syndrome_count; ++i) {
        if (S[i] != gf(0)) {
            clean = false;
            break;
        }
    }
    if (clean)
        return true;
    poly<gf> lambda;
    if (!berlekamp_massey(S, syndrome_count, lambda))
        return false;
    if (lambda.degree > config.T)
        return false;
    int positions[poly<gf>::MAX_TERMS];
    int count = 0;
    if (!chien_search(lambda, config.N, positions, count))
        return false;
    for (int i = 0; i < count; ++i) {
        int offset = config.N - 1 - positions[i];
        received[offset >> 3] ^= 1u << (7 - (offset & 7));
    }
    syndromes(received, S, config);
    for (int i = 0; i < syndrome_count; ++i) {
        if (S[i] != gf(0))
            return false;
    }
    return true;
}

template<const bch_config& cfg>
struct bch_encoder:runnable {
    bch_encoder(scheduler* sch, pipebuf<u8>& _in, pipebuf<u8>& _out):
    runnable(sch, "BCH_ENCODER"), in(_in), out(_out){
        out.buf.min_write = (cfg.N + 14) >> 3;
}

    void run() override {

        const int code_bytes = cfg.code_bytes();
        

        size_t messages = in.readable() / cfg.K;
        size_t codes = out.writable() / cfg.N;
        size_t count = messages < codes ? messages : codes;

        u8 code[code_bytes] = {};
        for (size_t i = 0; i < count; ++i) {

            in.read_bits(code, cfg.K);
            bch_encode(code, cfg);
            out.write(code, cfg.N);
        }
    }

private:
    pipereader_bit in;
    pipewriter_bit out;
};

template<const bch_config& cfg>
struct bch_decoder : runnable {
    using gf = GF<cfg.M, cfg.primitive>;

    bch_decoder(scheduler* sch,pipebuf<u8>& _in, pipebuf<u8>& _out):
    runnable(sch, "BCH_DECODER"),in(_in),out(_out){
    out.buf.min_write = (cfg.K + 14) >> 3;
}

    void run() override {
        const int bytes = cfg.code_bytes();
        u8 received[bytes] = {};

        size_t codes = in.readable() / cfg.N;
        size_t messages = out.writable() / cfg.K;

        size_t count = codes < messages? codes : messages;
        
        for (size_t i = 0; i < count; ++i) {
            in.read_bits(received, cfg.N);
            bch_decode<gf>(received, cfg);
            out.write(received, cfg.K);
        }

    }

private:
    pipereader_bit in;
    pipewriter_bit out;
};

}
#endif
