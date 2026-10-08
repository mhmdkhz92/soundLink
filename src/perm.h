#ifndef SOUNDLINK_PERM_H
#define SOUNDLINK_PERM_H

#include "framework.h"
#include "bch.h"
#include <vector>

namespace soundlink {
constexpr uint32_t SYNC_WORD = 0x1ACFFC1D;
// Each row holds one packed BCH codeword; complete matrices are byte-aligned.
template<size_t Width, size_t Height>
struct interleaver : runnable {
    static_assert(Height > 0 && Height % 8 == 0, "Interleaver height must be a multiple of eight");
    size_t block_bytes = Width * Height / 8;
    size_t sync_bytes = sizeof(SYNC_WORD);
    size_t frame_bytes = sync_bytes + block_bytes;
    interleaver(scheduler* sch, pipebuf<u8>& _in, pipebuf<u8>& _out)
        : runnable(sch, "INTERLEAVER"), in(_in), out(_out, frame_bytes) {}

    void run() override {
        size_t blocks = in.readable() / block_bytes;
        size_t space = out.writable() / frame_bytes;
        if (blocks > space)
            blocks = space;

        for (size_t block = 0; block < blocks; ++block) {
            const u8* src = in.rd();
            u8* dst = out.wr();
            // Serialize MSB-first, independent of host byte order.
            for (size_t i = 0; i < sync_bytes; ++i)
                dst[i] = (u8)(SYNC_WORD >> (8 * (sync_bytes - 1 - i)));
            u8* payload = dst + sync_bytes;
            memset(payload, 0, block_bytes);

            // Input fills rows; output visits columns, MSB-first.
            size_t bit = 0;
            for (size_t column = 0; column < Width; ++column) {
                for (size_t row = 0; row < Height; ++row, ++bit) {
                    const size_t input_bit = row * Width + column;
                    if (src[input_bit / 8] & (0x80u >> (input_bit % 8)))
                        payload[bit / 8] |= 0x80u >> (bit % 8);
                }
            }

            in.read(block_bytes);
            out.written(frame_bytes);
        }
    }

private:
    pipereader<u8> in;
    pipewriter<u8> out;
};

template<size_t Width, size_t Height>
struct deinterleaver : runnable {
    static_assert(Height > 0 && Height % 8 == 0, "Deinterleaver height must be  multiple of eight");

    size_t block_bits = Width * Height;
    size_t block_bytes = block_bits/8;
    deinterleaver(scheduler* sch, pipebuf<u8>& _in, pipebuf<u8>& _out)
        :runnable(sch, "DEINTERLEAVER"),
        sync(0),
        check(0),
        window(0),
        src(block_bytes),
        in(_in),
        out(_out, block_bytes){}

    void run() override {
        while(1){
            if(!sync) sync_acquisition();
            if(!sync) return;
            if(!check){
                if(in.readable() < 32) return;
                markerCheck();
                if(!sync) continue;
            }
            if(in.readable() < block_bits || out.writable() < block_bytes)
                return;

            for(size_t i = 0; i < block_bytes; ++i)
                in.read_bits(&src[i], 8);
            u8* dst = out.wr();
            memset(dst, 0, block_bytes);

            // Input fills columns; output restores rows, MSB-first.
            size_t bit = 0;
            for (size_t column = 0; column < Width; ++column) {
                for (size_t row = 0; row < Height; ++row, ++bit) {
                    size_t output_bit = row * Width + column;
                    if (src[bit / 8] & (0x80u >> (bit % 8)))
                        dst[output_bit / 8] |= 0x80u >> (output_bit % 8);
                }
            }

            out.written(block_bytes);
            check = 0;
        }
    }
    void sync_acquisition(){
        if(!sync){
            while(!sync && in.readable()){
                u8 packed;
                in.read_bits(&packed, 1);
                uint32_t bit = packed >> 7;
                window = (window << 1) | bit;
                if (window == SYNC_WORD){
                    check = 1;
                    sync = 4;
                    break;
                }
            }
        }
    }
    void markerCheck(){
        if(sync && in.readable() >= 32){
            u8 packed;
            for (int i = 0; i < 4; ++i){
                in.read_bits(&packed, 8);
                window = (window << 8) | packed;
            }
            if (window == SYNC_WORD) sync = 4;
            else --sync;
            check = sync ? 1 : 0;
        }
    }

private:
    u8 sync, check;
    uint32_t window;
    std::vector<u8> src;

    pipereader_bit in;
    pipewriter<u8> out;
};

}

#endif
