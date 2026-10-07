#ifndef SOUNDLINK_GENERIC_H
#define SOUNDLINK_GENERIC_H

#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

namespace soundlink {

//////////////////////////////////////////////////////////////////////
// Simple blocks
//////////////////////////////////////////////////////////////////////

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