#ifndef SOUNDLINK_FRAMEWORK_H
#define SOUNDLINK_FRAMEWORK_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <complex>
#include <math.h>


namespace soundlink{
    void fatal(const char *s){
        perror(s);exit(1);
    }
    void fail(const char *s){
        fprintf(stderr, "** %s\n", s);exit(1);
    }
    //////////////////////////////////////////////////////////////////////
    // DSP framework
    //////////////////////////////////////////////////////////////////////
    
    // [buff] defines the storage type, scalar or complex 
    // [pipebuf] is a FIFO buffer with multiple readers.
    // [pipewriter] is a client-side hook for writing into a [pipebuf].
    // [pipereader] is a client-side hook reading from a [pipebuf].
    // [runnable] is anything that moves data between [pipebufs].
    // [scheduler] is a global context which invokes [runnables] until fixpoint.
    static const int MAX_PIPES = 64;
    static const int MAX_RUNNABLES = 64;
    static const int MAX_READERS = 8;

    struct pipebuf_common{
        virtual long long hash() = 0;
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
            unsigned long long prev_hash = 0;
            while (1){
                step();
                unsigned long long h = hash();
                if ( h == prev_hash ) break;
                prev_hash = h;
            }
        }

        void shutdown() {
        for ( int i=0; i<nrunnables; ++i )
        runnables[i]->shutdown();
        }
        unsigned long long hash(){
        unsigned long long h = 0;
        for ( int i=0; i<npipes; ++i )
            h += (1+i)*pipes[i]->hash();
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
        pipebuf(scheduler *sch, const char *name, unsigned long size): pipebuf_common(name),
	      re(new T[size]), nrd(0), wr(re), end(re+size),
	      min_write(1), total_written(0), total_read(0) {
            sch->add_pipe(this);
        }
        pipebuf(const char *name, unsigned long size): pipebuf_common(name),
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
            memmove(re, rd, (wr-rd)*sizeof(T));
            if(im){
                memmove(im, im + (rd - re), (wr-rd)*sizeof(T));
            }
            wr -= rd - re;
            for ( int i=0; i<nrd; ++i ) 
                rds[i] -= rd - re;
        }
        long long hash() {
            return total_written + total_read;
        }
        ~pipebuf() {
            delete[] re;
            delete[] im;
        }
        unsigned long min_write;
        unsigned long total_written, total_read;
    };

    template<typename T>
    struct pipebuf_c:pipebuf<T>{
        pipebuf_c(scheduler* sch, const char *name, unsigned long size):
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
        complex_ref<T> operator[](unsigned long i) const {
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
    };

    template<typename T>
    struct pipewriter {
        pipebuf<T> &buf;
        pipewriter(pipebuf<T> &_buf, unsigned long min_write=1):buf(_buf){
        if ( min_write > buf.min_write ) 
            buf.min_write = min_write;
        }
        // Return number of items writable at this->wr, 0 if full.
        unsigned long writable() {
            if ( buf.end-buf.wr < buf.min_write ) 
                buf.pack();
                return buf.end - buf.wr;
        }
        T *wr(){
          return buf.wr;
        }
        complex_view<T> wr_c(){
            if(!buf.im)
                fail("complex op on scalar buffer");
            return{buf.wr, buf.im + (buf.wr - buf.re)};
        }
        void written(unsigned long n) {
            if ( buf.wr+n > buf.end ) {
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
        unsigned long readable(){
            return buf.wr - buf.rds[id];
        }
        T* rd(){
            return buf.rds[id];
        }
        complex_view<T> rd_c(){
            if(!buf.im)
                fail("complex op on scalar buffer");
            return{buf.rds[id], buf.im + (buf.rds[id] - buf.re)};
        }
        void read(unsigned long n) {
            if ( buf.rds[id]+n > buf.wr ) {
                fprintf(stderr, "Bug: underflow from %s\n", buf.name);
                exit(1);
            }
            buf.rds[id] += n;
            buf.total_read += n;
        }
    };
    typedef uint32_t u32;
    typedef uint16_t u16;
    typedef uint8_t u8;
    typedef std::complex<float> cf32;
    typedef complex_view<float> cv32;
}
#endif  