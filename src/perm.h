#ifndef SOUNDLINK_PERM_H
#define SOUNDLINK_PERM_H

#include "framework.h"
#include "bch.h"

namespace soundlink {

// Width is measured in bits and is a multiple of eight.
template<unsigned Width, unsigned Height>
struct interleaver : runnable {

    unsigned block_bytes = (Width / 8) * Height;
    interleaver(scheduler* sch, pipebuf<u8>& _in, pipebuf<u8>& _out)
        : runnable(sch, "INTERLEAVER"), in(_in), out(_out, block_bytes) {}

    void run() override {
        unsigned long blocks = in.readable() / block_bytes;
        unsigned long space = out.writable() / block_bytes;
        if (blocks > space)
            blocks = space;

        for (unsigned long block = 0; block < blocks; ++block) {
            const u8* src = in.rd();
            u8* dst = out.wr();
            memset(dst, 0, block_bytes);

            // Input fills rows; output visits columns, MSB-first.
            unsigned bit = 0;
            for (unsigned column = 0; column < Width; ++column) {
                for (unsigned row = 0; row < Height; ++row, ++bit) {
                    if (src[row * (Width / 8) + column / 8] & (0x80u >> (column % 8)))
                        dst[bit / 8] |= 0x80u >> (bit % 8);
                }
            }

            in.read(block_bytes);
            out.written(block_bytes);
        }
    }

private:
    pipereader<u8> in;
    pipewriter<u8> out;
};

template<unsigned Width, unsigned Height>
struct deinterleaver : runnable {

    unsigned block_bytes = (Width / 8) * Height;
    deinterleaver(scheduler* sch, pipebuf<u8>& _in, pipebuf<u8>& _out)
        : runnable(sch, "DEINTERLEAVER"), in(_in), out(_out, block_bytes) {}

    void run() override {
        unsigned long blocks = in.readable() / block_bytes;
        unsigned long space = out.writable() / block_bytes;
        if (blocks > space)
            blocks = space;

        for (unsigned long block = 0; block < blocks; ++block) {
            const u8* src = in.rd();
            u8* dst = out.wr();
            memset(dst, 0, block_bytes);

            // Input fills columns; output restores rows, MSB-first.
            unsigned bit = 0;
            for (unsigned column = 0; column < Width; ++column) {
                for (unsigned row = 0; row < Height; ++row, ++bit) {
                    if (src[bit / 8] & (0x80u >> (bit % 8)))
                        dst[row * (Width / 8) + column / 8] |= 0x80u >> (column % 8);
                }
            }

            in.read(block_bytes);
            out.written(block_bytes);
        }
    }

private:
    pipereader<u8> in;
    pipewriter<u8> out;
};

}

#endif
