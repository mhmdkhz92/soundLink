#ifndef SOUNDLINK_FRAMEWORK_H
#define SOUNDLINK_FRAMEWORK_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <complex>
#include <cstddef>
#include <vector>
#include <string>

namespace soundlink{
inline void fatal(const char *s){
        perror(s);exit(1);
}
inline void fail(const char *s){
        fprintf(stderr, "** %s\n", s);exit(1);
}
//////////////////////////////////////////////////////////////////////
// DSP framework
//////////////////////////////////////////////////////////////////////

static const int MAX_PIPES = 64;
static const int MAX_RUNNABLES = 64;
static const int MAX_READERS = 8;
static const uint16_t NSYM_PER_RB = 6;

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;
typedef std::complex<float> cf32;

struct pipebuf_common{
    virtual uint64_t hash() = 0;
    const char *name;
    pipebuf_common(const char *_name) : name(_name){}
};

struct runnable_common{
    const char *name;
    runnable_common(const char *_name) : name(_name){}
    virtual void run(){};
    virtual void shutdown(){};
};

struct scheduler{
    pipebuf_common *pipes[MAX_PIPES];
    int npipes;
    runnable_common *runnables[MAX_RUNNABLES];
    int nrunnables;
    scheduler(): npipes(0), nrunnables(0){}

    void add_pipe(pipebuf_common *p) {
        if (npipes == MAX_PIPES )
            fail("MAX_PIPES");
        pipes[npipes++] = p;
    }
    void add_runnable(runnable_common *r){
        if ( nrunnables == MAX_RUNNABLES )
            fail("MAX_RUNNABLES");
        runnables[nrunnables++] = r;
    }

    void step() {
        for(int i = 0; i < nrunnables; ++i)
            runnables[i]->run();
    }
    void run(){
        uint64_t prev_hash = 0;
        while (1){
            step();
            uint64_t h = hash();
            if ( h == prev_hash ) break;
            prev_hash = h;
        }
    }

    void shutdown() {
    for ( int i=0; i<nrunnables; ++i )
    runnables[i]->shutdown();
    }
    uint64_t hash(){
    uint64_t h = 0;
    for ( int i=0; i<npipes; ++i )
        h += (uint64_t)(1+i)*pipes[i]->hash();
    return h;
    }
};

struct runnable:runnable_common {
    runnable(scheduler *_sch, const char *name):runnable_common(name), sch(_sch) {
    sch->add_runnable(this);
    }
protected:
    scheduler *sch;
};
template<typename T>
struct pipebuf : pipebuf_common {
    T *re;
    T *im = nullptr;
    T *rds[MAX_READERS];
    int nrd;
    T *wr;
    T *end;
    pipebuf(scheduler *sch, const char *name, size_t size): pipebuf_common(name),
        re(new T[size]), nrd(0), wr(re), end(re+size),
        min_write(1), total_written(0), total_read(0) {
        sch->add_pipe(this);
    }
    // constructor for complex buffers
    pipebuf(const char *name, size_t size): pipebuf_common(name),
        re(new T[size]), nrd(0), wr(re), end(re+size),
        min_write(1), total_written(0), total_read(0) {
        im = new T[size];
    }
    int add_reader() {
        if ( nrd == MAX_READERS )
            fail("too many readers");
        rds[nrd] = wr;
        return nrd++;
    }
    void pack() {
        T *rd = wr;
        for ( int i=0; i<nrd; ++i )
            if ( rds[i] < rd )
                rd = rds[i];
        const size_t retained = (size_t)(wr - rd);
        memmove(re, rd, retained * sizeof(T));
        if(im){
            memmove(im, im + (rd - re), retained * sizeof(T));
        }
        wr -= rd - re;
        for ( int i=0; i<nrd; ++i ) 
            rds[i] -= rd - re;
    }
    uint64_t hash() {
        return total_written + total_read;
    }
    ~pipebuf() {
        delete[] re;
        delete[] im;
    }
    size_t min_write;
    uint64_t total_written, total_read;
};

template<typename T>
struct pipebuf_c:pipebuf<T>{
    pipebuf_c(scheduler* sch, const char *name, size_t size):
    pipebuf<T>(name, size){
        sch->add_pipe(this);
    }
};


template<typename T>
struct complex_ref {
    T& re;
    T& im;
    // Write a complex value into the two arrays.
    complex_ref& operator=(const std::complex<T>& value) {
        re = value.real();
        im = value.imag();
        return *this;
    }

    // Allow a[i] = a[j].
    complex_ref& operator=(const complex_ref& other) {
        re = other.re;
        im = other.im;
        return *this;
    }

    // convert to std::complex<T> on demand
    operator std::complex<T>() const{
        return {re, im};
    }
};


template<typename T>
struct complex_view {
    T* re;
    T* im;
    complex_ref<T> operator[](size_t i) const {
        return {re[i], im[i]};
    }
    complex_ref<T> operator*() const {
        return {re[0], im[0]};
    }
    complex_view operator+(std::ptrdiff_t offset) const {
        return {re + offset, im + offset};
    }
    complex_view& operator+=(std::ptrdiff_t offset) {
        re += offset;
        im += offset;
        return *this;
    }
}; typedef complex_view<float> cv32;


template<typename T>
struct pipewriter {
    pipebuf<T> &buf;
    pipewriter(pipebuf<T> &_buf, size_t min_write=1):buf(_buf){
    if ( min_write > buf.min_write ) 
        buf.min_write = min_write;
    }
    // Return number of items writable at this->wr, 0 if full.
    size_t writable() {
        if ( (size_t)(buf.end - buf.wr) < buf.min_write ) 
            buf.pack();
        return (size_t)(buf.end - buf.wr);
    }
    T *wr(){
        return buf.wr;
    }
    complex_view<T> wr_c(){
        if(!buf.im)
            fail("complex op on scalar buffer");
        return{buf.wr, buf.im + (buf.wr - buf.re)};
    }
    void written(size_t n) {
        if ( n > (size_t)(buf.end - buf.wr) ) {
                fprintf(stderr, "Bug: overflow to %s\n", buf.name);
                exit(1);
        }
        buf.wr += n;
        buf.total_written += n;
    }
    inline void write(const T &e) {
        *wr() = e;
        if(buf.im)
            *wr_c().im = 0;
        written(1);
    }
    inline void write(const std::complex<T>& e){
        if(!buf.im)
            fail("complex op on scalar buffer");
        *wr_c() = e;
        written(1);
    }
};

template<typename T>
struct pipereader {
    pipebuf<T> &buf;
    int id;
    pipereader(pipebuf<T> &_buf) : buf(_buf), id(_buf.add_reader()) { }
    size_t readable(){
        return (size_t)(buf.wr - buf.rds[id]);
    }
    T* rd(){
        return buf.rds[id];
    }
    complex_view<T> rd_c(){
        if(!buf.im)
            fail("complex op on scalar buffer");
        return{buf.rds[id], buf.im + (buf.rds[id] - buf.re)};
    }
    void read(size_t n) {
        if ( n > (size_t)(buf.wr - buf.rds[id]) ) {
            fprintf(stderr, "Bug: underflow from %s\n", buf.name);
            exit(1);
        }
        buf.rds[id] += n;
        buf.total_read += n;
    }
};

struct pipereader_bit{
    pipebuf<u8>& buf;
    int id;
    pipereader_bit(pipebuf<u8> &_buf) : buf(_buf), id(_buf.add_reader()) { 
        held = 0;
        nheld = 0;
    }
    size_t readable(){
        return 8 * (size_t)(buf.wr - buf.rds[id]) + nheld;
    }
    void read_bits(u8* destination, unsigned nbits){
        if (nbits == 0)
            return;
        size_t count = 0;
        while(nbits>= 8){
            if(nheld < 8)
                fill_held();
            destination[count++] = (held >> (nheld - 8)) & 0xFF;
            nbits -= 8;
            nheld -= 8;
        }
        if(nbits){
            if(nheld < nbits)
                fill_held();
            destination[count] = (u8)((held >> (nheld - nbits)) & ((1u << nbits) - 1));
            destination[count] <<= (8 - nbits);
            nheld -= nbits;
        }
    }
    void fill_held(){
        size_t bytes = (size_t)(buf.wr - buf.rds[id]);
        if (bytes > 7)
            bytes = 7;
        const u8* rd = buf.rds[id];
        for (size_t i = 0; i < bytes; ++i)
            held = (held << 8) | uint64_t{rd[i]};
        nheld += bytes * 8;
        read(bytes);
    }
    void read(size_t n) {
        if ( n > (size_t)(buf.wr - buf.rds[id]) ) {
            fprintf(stderr, "Bug: underflow from %s\n", buf.name);
            exit(1);
        }
        buf.rds[id] += n;
        buf.total_read += n;
    }


private:
    uint64_t held;
    size_t nheld;
};

struct pipewriter_bit{
    pipebuf<u8>& buf;

    pipewriter_bit(pipebuf<u8> &_buf, size_t min_write = 1):buf(_buf){
    if ( min_write > buf.min_write) {
        buf.min_write = min_write;
    }
        held = 0;
        nheld = 0;
    }
    size_t writable(){
        if ( (size_t)(buf.end - buf.wr) < buf.min_write) 
            buf.pack();
        const size_t capacity = 8 * (size_t)(buf.end - buf.wr);
        return capacity >= nheld ? capacity - nheld : 0;
    }
    void write(u8* src, unsigned nbits){
        size_t count = 0;
        unsigned width = 0;
        while(nbits){
            width = nbits < 8 ? nbits: 8;
            held = (uint16_t)((held << width) | (src[count ++] >> (8 - width)));
            nheld += width;
            nbits -= width;
            if(nheld >> 3){
                *buf.wr = (u8)(held >> (nheld - 8) & 0xFF);
                written(1);
                nheld -= 8;
            }
        }

    }

    void written(size_t n) {
        if ( n > (size_t)(buf.end - buf.wr) ) {
                fprintf(stderr, "Bug: overflow to %s\n", buf.name);
                exit(1);
        }
        buf.wr += n;
        buf.total_written += n;
    }

private:
    uint16_t held;
    size_t nheld;
};


//////////////////////////////////////////////////////////////////////
// shared definitions and interfaces
//////////////////////////////////////////////////////////////////////
enum class device{
    sender, receiver
};
enum BW{
    BW1_4, BW3, BW5, BW10};
enum modulation{
    BPSK = 1, QPSK = 2, QAM16 = 4, QAM64 = 6};

enum code_rate {
	FEC12, FEC23, FEC34, FEC56, FEC78,
};

struct link_cfg{
    device dev;
    BW bw;                  // bandwidth in BW enum
    int fc;                 // center frequency
    float bandwidth;        // bandwidth in Hz
    float sr;               // sample rate
    u16 nRB;                // number of resource blocks
    u16 nSC;                // number of active subcarriers
    u16 nFFT;               // number of FFT points
    u16 nSym;               // number of symbols ber slot
    u16 cp;                 // number of cyclic prefix points
    code_rate cr;           // convolutional rate
    modulation mod;           // modulation type
    link_cfg(device _dev, int _fc, BW _bw, code_rate _cr, modulation _m = QPSK)
    :dev(_dev), fc(_fc), nSym(NSYM_PER_RB), cr(_cr), mod(_m){
        switch (_bw){
        case BW1_4:
            bw = BW1_4; bandwidth = 1.4e3; nRB = 6; nFFT = 128;
            break;
        case BW3:
            bw = BW3; bandwidth = 3e3; nRB = 15; nFFT = 256;
            break;
        case BW5:
            bw = BW5; bandwidth = 5e3; nRB = 25; nFFT = 512;
            break;
        case BW10:
            bw = BW10; bandwidth = 10e3; nRB = 50; nFFT = 1024;
            break;
        }
        nSC = nRB * 12;
        sr = nFFT * 15;
        cp = nFFT / 4;
    }
};

struct tx_channel {
    const char* name;

protected:
    scheduler sch;
    const link_cfg cfg;
    pipebuf<float> audio;
    pipereader<float> output;

public:
    tx_channel(const char* name, const link_cfg& _cfg)
    : name(name), cfg(_cfg), audio(&sch, "audio", 24000), output(audio) {}

    const link_cfg& config() { return cfg; }
    size_t available() { return output.readable(); }
    const float* samples() { return output.rd(); }
    void consume(size_t count) { output.read(count); }

    virtual void set_sltn(u8 n) = 0;
    virtual void run() = 0;
    virtual ~tx_channel() = default;
};

struct frame_timing;
typedef void (*callback)();

template<typename T>
struct sample_request {
    T* points;
    size_t capacity;
    size_t* written;
    callback ready = nullptr;
};

struct rx_channel {
    const char* name;
    float snr = 0;
    bool snr_valid = false;

protected:
    scheduler sch;
    const link_cfg cfg;
    pipebuf<float> audio;
    pipewriter<float> input;

public:
    rx_channel(const char* name, const link_cfg& _cfg)
    : name(name), cfg(_cfg), audio(&sch, "audio", 48000), input(audio) {}

    const link_cfg& config() { return cfg; }
    size_t writable() { return input.writable(); }
    float* samples() { return input.wr(); }
    void written(size_t count) { input.written(count); }

    virtual void step() = 0;
    virtual bool get_stamp(frame_timing& ft) = 0;
    virtual void clear_stamp() = 0;
    // information:
    virtual bool estimate_snr(float& db) = 0;
    virtual size_t received_bytes() { return 0; }
    virtual void constellation(sample_request<cf32>) = 0;

    virtual ~rx_channel() = default;
};
struct channel_info {
    uint8_t fc;
    uint8_t code_rate;
    uint8_t bandwidth;
    uint8_t mod;
};
using channel_vec = std::vector<channel_info>;
using cfg_vec = std::vector<link_cfg>;

}
#endif  
