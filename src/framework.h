#ifndef SOUNDLINK_FRAMEWORK_H
#define SOUNDLINK_FRAMEWORK_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <complex>
#include <math.h>


namespace soundlink {
  
  void fatal(const char *s) { perror(s); exit(1); }
  void fail(const char *s) { fprintf(stderr, "** %s\n", s); exit(1); }
  
  //////////////////////////////////////////////////////////////////////
  // DSP framework
  //////////////////////////////////////////////////////////////////////
  
  // [pipebuf] is a FIFO buffer with multiple readers.
  // [pipewriter] is a client-side hook for writing into a [pipebuf].
  // [pipereader] is a client-side hook reading from a [pipebuf].
  // [runnable] is anything that moves data between [pipebufs].
  // [scheduler] is a global context which invokes [runnables] until fixpoint.
  
  static const int MAX_PIPES = 64;
  static const int MAX_RUNNABLES = 64;
  static const int MAX_READERS = 8;
  
  struct pipebuf_common {
    virtual int sizeofT() { return 0; }
    virtual long long hash() { return 0; }
    virtual void dump(size_t *total_bufs) { }
    const char *name;
    pipebuf_common(const char *_name) : name(_name) { }
  };

  struct runnable_common {
    const char *name;
    runnable_common(const char *_name) : name(_name) { }
    virtual void run() { }
    virtual void shutdown() { }
#ifdef DEBUG
    ~runnable_common() { fprintf(stderr, "Deallocating %s !\n", name); }
#endif
  };


  struct scheduler {
    pipebuf_common *pipes[MAX_PIPES];
    int npipes;
    runnable_common *runnables[MAX_RUNNABLES];
    int nrunnables;

    scheduler()
      : npipes(0), nrunnables(0){
    }
    void add_pipe(pipebuf_common *p) {
      if ( npipes == MAX_PIPES ) fail("MAX_PIPES");
      pipes[npipes++] = p;
    }
    void add_runnable(runnable_common *r) {
      if ( nrunnables == MAX_RUNNABLES ) fail("MAX_RUNNABLES");
      runnables[nrunnables++] = r;
    }
    void step() {
      for ( int i=0; i<nrunnables; ++i )
	runnables[i]->run();
    }
    void run() {
      unsigned long long prev_hash = 0;
      while ( 1 ) {
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
    unsigned long long hash() {
      unsigned long long h = 0;
      for ( int i=0; i<npipes; ++i ) h += (1+i)*pipes[i]->hash();
      return h;
    }
    
    void dump() {
      fprintf(stderr, "\n");
      size_t total_bufs = 0;
      for ( int i=0; i<npipes; ++i ) pipes[i]->dump(&total_bufs);
      fprintf(stderr, "Total buffer memory: %ld KiB\n",
	      (unsigned long)total_bufs/1024);
    }
  };
  
  struct runnable : runnable_common {
    runnable(scheduler *_sch, const char *name)
      : runnable_common(name), sch(_sch) {
      sch->add_runnable(this);
    }
  protected:
    scheduler *sch;
  };
  
  template<typename T>
  struct pipebuf : pipebuf_common {
    T *buf;
    T *rds[MAX_READERS];
    int nrd;
    T *wr;
    T *end;
    int sizeofT() { return sizeof(T); }
    pipebuf(scheduler *sch, const char *name, unsigned long size)
      : pipebuf_common(name),
	buf(new T[size]), nrd(0), wr(buf), end(buf+size),
	min_write(1),
	total_written(0), total_read(0) {
      sch->add_pipe(this);
    }
    int add_reader() {
      if ( nrd == MAX_READERS ) fail("too many readers");
      rds[nrd] = wr;
      return nrd++;
    }
    void pack() {
      T *rd = wr;
      for ( int i=0; i<nrd; ++i ) if ( rds[i] < rd ) rd = rds[i];
      memmove(buf, rd, (wr-rd)*sizeof(T));
      wr -= rd - buf;
      for ( int i=0; i<nrd; ++i ) rds[i] -= rd - buf;
    }
    long long hash() {
      return total_written + total_read;
    }
    void dump(size_t *total_bufs) {
      if ( total_written < 10000 ) 
	fprintf(stderr, ".%-16s : %4ld/%4ld", name,
		total_read, total_written);
      else if ( total_written < 1000000 ) 
	fprintf(stderr, ".%-16s : %3ldk/%3ldk", name,
		total_read/1000, total_written/1000);
      else 
	fprintf(stderr, ".%-16s : %3ldM/%3ldM", name,
		total_read/1000000, total_written/1000000);
      *total_bufs += (end-buf) * sizeof(T);
      unsigned long nw = end - wr;
      fprintf(stderr, " %6ld writable %c,", nw, (nw<min_write)?'!':' ');
      T *rd = wr;
      for ( int j=0; j<nrd; ++j ) if ( rds[j] < rd ) rd = rds[j];
      fprintf(stderr, " %6d unread (", (int)(wr-rd));
      for ( int j=0; j<nrd; ++j )
	fprintf(stderr, " %d", (int)(wr-rds[j]));
      fprintf(stderr, " )\n");
    }
    unsigned long min_write;
    unsigned long total_written, total_read;
#ifdef DEBUG
    ~pipebuf() { fprintf(stderr, "Deallocating %s !\n", name); }
#endif
  };
  
  template<typename T>
  struct pipewriter {
    pipebuf<T> &buf;
    pipewriter(pipebuf<T> &_buf, unsigned long min_write=1)
      : buf(_buf) {
      if ( min_write > buf.min_write ) buf.min_write = min_write;
    }
    // Return number of items writable at this->wr, 0 if full.
    unsigned long writable() {
      if ( buf.end-buf.wr < buf.min_write ) buf.pack();
      return buf.end - buf.wr;
    }
    T *wr() { return buf.wr; }
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
      written(1);
    }
  };


  template<typename T>
  struct pipereader {
    pipebuf<T> &buf;
    int id;
    pipereader(pipebuf<T> &_buf) : buf(_buf), id(_buf.add_reader()) { }
    unsigned long readable() { return buf.wr - buf.rds[id]; }
    T *rd() { return buf.rds[id]; }
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
  

}  // namespace

#endif  