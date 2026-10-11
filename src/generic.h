#ifndef SOUNDLINK_GENERIC_H
#define SOUNDLINK_GENERIC_H

#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include "framework.h"
#include <algorithm>

namespace soundlink {


// Low 23 bits hold the LFSR; bits 23..30 collect the next output byte.
// Fibonacci polynomial x^23 + x^18 + 1, period 8388607 bits. Output is MSB-first.
struct prbs_generator : runnable {
  prbs_generator(scheduler *sch, pipebuf<u8> &_out, uint32_t seed = 1)
    : runnable(sch, "PRBS_GENERATOR"),
    reg(seed),
    out(_out){
    if (seed == 0 || seed > 0x7FFFFFu)
      fail("PRBS seed must be a nonzero 23-bit value");
  }
  void run() override {
    size_t bytes = out.writable();
    for (size_t i = 0; i < bytes; ++i) {
      for (unsigned bit = 0; bit < 8; ++bit) {
        uint32_t feedback = ((reg >> 22) ^ (reg >> 17)) & 1u;
        reg = (reg << 1) | feedback;
      }
      out.write((u8)(reg >> 23));
    }
  }
private:
  uint32_t reg;
  pipewriter<u8> out;
};

// Acquire from the first 23 bits of three bytes, then validate four bytes.
// Acquisition bits are excluded from BER. Reference state never follows RX errors.
struct ber_track : runnable {
  bool locked = false;
  uint64_t compared_bits = 0;
  uint64_t error_bits = 0;
  uint64_t acquisitions = 0;

  ber_track(scheduler *sch, pipebuf<u8> &_in)
    : runnable(sch, "BER_TRACK"), in(_in) {}

  double ber() {
    return compared_bits ? (double)error_bits / (double)compared_bits : 0.0;
  }
  void run() override {
    if(!locked) lock();
    if(!locked) return;
    size_t size = in.readable();
    u8* rd = in.rd();
    for(size_t i = 0; i < size; ++i){
      uint32_t difference = (uint32_t)(rd[i] ^ next_byte(reg));
      unsigned errors = 0;
      while(difference){
        difference &= difference - 1;
        ++errors;
      }
      compared_bits += 8;
      error_bits += errors;
      recent_bits += 8;
      recent_errors += errors;

      // A sustained >25% mismatch suggests wrong phase rather than a few errors.
      if(recent_bits == 2048){
        locked = recent_errors <= 512;
        recent_bits = recent_errors = 0;
        if(!locked){
          in.read(i + 1);
          return;
        }
      }
    }
    in.read(size);

  }
private:
  uint32_t reg = 0;
  unsigned recent_bits = 0, recent_errors = 0;
  pipereader<u8> in;

  u8 next_byte(uint32_t& state){
      for(unsigned i = 0; i < 8; ++i){
          uint32_t feedback = ((state >> 22) ^ (state >> 17)) & 1u;
          state = (state << 1) | feedback;
      }
      return (u8)(state >> 23);
  }
  void lock(){
    while(in.readable() >= 7){
      u8* src = in.rd();
      uint32_t candidate = 0;
      for(unsigned i = 0; i < 3; ++i)
        candidate = (candidate << 8) | src[i];
      candidate >>= 1;
      for(unsigned i = 0; i < 3; ++i)
        next_byte(candidate);

      bool match = candidate != 0;
      for(size_t i = 3; match && i < 7; ++i)
        match = next_byte(candidate) == src[i];
      if(!match){
        in.read(1);
        continue;
      }
      reg = candidate;
      in.read(7);
      locked = true;
      ++acquisitions;
      recent_bits = recent_errors = 0;
      break;
    }
  }
};

// [file_reader] reads raw data from a file descriptor into a [pipebuf].
// If the file descriptor is seekable, data can be looped.

template<typename T>
struct file_reader : runnable {
  file_reader(scheduler *sch, int _fdin, pipebuf<T> &_out)
    : runnable(sch, _out.name),
      loop(false),
      filler(NULL),
      fdin(_fdin), out(_out)
  {
  }
  void run() {
    size_t size = out.writable() * sizeof(T);
    if ( ! size ) return;

  again:
    ssize_t nr = read(fdin, out.wr(), size);
    if ( nr<0 && errno==EWOULDBLOCK && filler ) {
      out.write(*filler);
      return;
    }
    if ( nr < 0 ) fatal("read");
    if ( ! nr ) {
      if ( ! loop ) return;
      off_t res = lseek(fdin, 0, SEEK_SET);
      if ( res == (off_t)-1 ) fatal("lseek");
      goto again;
    }

    // Always stop at element boundary (may block)
    size_t bytes_read = (size_t)nr;
    size_t partial = bytes_read % sizeof(T);
    size_t remain = partial ? sizeof(T)-partial : 0;
    while ( remain ) {
      ssize_t nr2 = read(fdin, (char*)out.wr()+bytes_read, remain);
      if ( nr2 <= 0 ) fatal("partial read");
      bytes_read += (size_t)nr2;
      remain -= (size_t)nr2;
    }

    out.written(bytes_read / sizeof(T));
  }
  bool loop;
  void set_realtime(T &_filler) {
    int flags = fcntl(fdin, F_GETFL);
    if ( fcntl(fdin, F_SETFL, flags|O_NONBLOCK) ) fatal("fcntl");
    filler = new T(_filler);
  }
private:
  T *filler;
  int fdin;
  pipewriter<T> out;
};

// [file_writer] writes raw data from a [pipebuf] to a file descriptor.

template<typename T>
struct file_writer : runnable {
  file_writer(scheduler *sch, pipebuf<T> &_in, int _fdout) :
    runnable(sch, _in.name),
    in(_in), fdout(_fdout) {
  }
  void run() {
    size_t size = in.readable() * sizeof(T);
    if ( ! size ) return;
    ssize_t nw = write(fdout, in.rd(), size);
    if ( ! nw ) fatal("pipe");
    if ( nw < 0 ) fatal("write");
    const size_t bytes_written = (size_t)nw;
    if ( bytes_written % sizeof(T) ) fatal("partial write");
    in.read(bytes_written/sizeof(T));
  }
private:
  pipereader<T> in;
  int fdout;
};


}  // namespace

#endif  // SOUNDLINK_GENERIC_H
