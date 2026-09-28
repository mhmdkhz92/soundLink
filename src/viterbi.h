#ifndef SOUNDLINK_VITERBI_H
#define SOUNDLINK_VITERBI_H

#include <stdio.h>
#include <stdint.h>
#include <algorithm>
#include <cstring>

#include "framework.h"
#include "puncture.h"

extern unsigned char d_Partab[];
const int POLYA = 0x4f;
const int POLYB = 0x6d;
typedef int (*punc_t)(const u8*, u8*, uint16_t&);
enum code_rate {
	FEC12, FEC23, FEC34, FEC56, FEC78,
};

#define BUTTERFLY(i, sym) {                              \
    uint32_t m0 = state[i]      + mets[sym];             \
    uint32_t m1 = state[i + 32] + mets[3 ^ (sym)];       \
                                                    	 \
    if (m0 <= m1) {                                      \
        next[2 * (i)] = m0;                              \
    } else {                                             \
        next[2 * (i)] = m1;                              \
        decisions |= uint64_t{1} << (2 * (i));           \
    }                                                    \
                                                         \
    uint32_t m2 = state[i]      + mets[3 ^ (sym)];       \
    uint32_t m3 = state[i + 32] + mets[sym];             \
                                                         \
    if (m2 <= m3) {                                      \
        next[2 * (i) + 1] = m2;                          \
    } else {                                             \
        next[2 * (i) + 1] = m3;                          \
        decisions |= uint64_t{1} << (2 * (i) + 1);       \
    }                                                    \
}
using namespace soundlink;
class viterbi :runnable {
private:
	int cr[2] = {};
	code_rate crate;
	uint32_t state0[64] = {};
	uint32_t state1[64] = {};
	uint32_t* state = state0, * next = state1;

	uint64_t trace_buffer[BUFFER_LEN] = {};

	u8 input_buffer[INPUT_BUFFER_EXT] = {};

	uint16_t punc_offset = 0; // offset in the input buffer
	uint16_t pos = 0; // position in the traceback buffer

	// allignment and rotation flags
	uint8_t punc_phase_num = 0;
	uint8_t sync_flag = 0;
	uint8_t skip_flag = 1;

	u8 decoded_bytes[OUTPUT_LEN_BYTES] = {};

	pipewriter<u8> out;
	pipereader<u8> in;
	punc_t puncturer;


	void decoder_reset() {
		for(int i=0; i < 64;  i++){
    		state0[i] = 0;
			state1[i] = 0;
		}
		state = state0;
		next = state1;
		punc_offset = 0;
		pos = 0;
		skip_flag = 1;
	}
	u8 encode(u8* symbols, u8* data, size_t nbytes, u8 encstate) {
		int i;
		while (nbytes-- != 0) {
			for (i = 7;i >= 0;i--) {
				encstate = (encstate << 1) | ((*data >> i) & 1);
				*symbols++ = d_Partab[encstate & POLYA];
				*symbols++ = d_Partab[encstate & POLYB];
			}
			data++;
		}
		return encstate;
	}
	inline void decode(u8* output_buffer) {

		int mets[4];
		for (int bitcnt = 0; bitcnt < INPUT_BUFFER; bitcnt += 2) {
			u8 A = input_buffer[bitcnt];
			u8 B = input_buffer[bitcnt + 1];
			if (A>>7&1){
				mets[0] = B;          
				mets[1] = (127 - B); 
				mets[2] = B;         
				mets[3] = (127 - B);
			}
			else if (B>>7&1){
				mets[0] = A;
				mets[1] = A;
				mets[2] = (127 - A);
				mets[3] = (127 - A);

			}
			else{
				mets[0] = A + B;          		  // expected 00
				mets[1] = A + (127 - B);  		  // expected 01
				mets[2] = (127 - A) + B;          // expected 10
				mets[3] = (127 - A) + (127 - B);  // expected 11
			}
			// ACS butterfly 
			uint64_t decisions = 0;
			BUTTERFLY(0,0);BUTTERFLY(1,2);BUTTERFLY(2,3);BUTTERFLY(3,1);
			BUTTERFLY(4,3);BUTTERFLY(5,1);BUTTERFLY(6,0);BUTTERFLY(7,2);
			BUTTERFLY(8,0);BUTTERFLY(9,2);BUTTERFLY(10,3);BUTTERFLY(11,1);
			BUTTERFLY(12,3);BUTTERFLY(13,1);BUTTERFLY(14,0);BUTTERFLY(15,2);
			BUTTERFLY(16,1);BUTTERFLY(17,3);BUTTERFLY(18,2);BUTTERFLY(19,0);
			BUTTERFLY(20,2);BUTTERFLY(21,0);BUTTERFLY(22,1);BUTTERFLY(23,3);
			BUTTERFLY(24,1);BUTTERFLY(25,3);BUTTERFLY(26,2);BUTTERFLY(27,0);
			BUTTERFLY(28,2);BUTTERFLY(29,0);BUTTERFLY(30,1);BUTTERFLY(31,3);


			// switch the "next pointer" and "state pointer" containers for the next iteration
			auto pointer = state;
			state = next;
			next = pointer;
			trace_buffer[pos] = decisions;
			pos = (pos + 1) & (BUFFER_LEN - 1);
		}
		std::memcpy(input_buffer, input_buffer + INPUT_BUFFER, punc_offset);
		// 2. TRACEBACK
		int32_t out_idx = OUTPUT_LEN_BYTES - 1;
		uint8_t acc = 0;
		uint32_t start_idx = pos - 1;
		
		uint8_t tr_state = 0; //min_index_64(stateptr);
		for (int i = 1; i < 64; ++i)
			if (state[i] < state[tr_state])
				tr_state = static_cast<uint8_t>(i);
		//convergence phase
		for (int t = 0; t < TRACEBACK; ++t) {
			uint32_t ring_index = (start_idx - t) & (BUFFER_LEN - 1);
			uint64_t stage_decisions = trace_buffer[ring_index];

			uint64_t D = (stage_decisions >> tr_state) & 1;
			tr_state = (tr_state >> 1) | (D << 5);
		}

		// collect phase
		for (int t = TRACEBACK; t < BUFFER_LEN; ++t) {
			uint32_t ring_index = (start_idx - t) & (BUFFER_LEN - 1);
			uint64_t stage_decisions = trace_buffer[ring_index];

			uint64_t D = (stage_decisions >> tr_state) & 1;

			acc = (acc >> 1) | ((tr_state & 1) << 7);
			tr_state = (tr_state >> 1) | (D << 5);

			if ((t & 7) == 7) {
				decoded_bytes[out_idx] = acc;
				out_idx--;
			}
		}
		std::memcpy(output_buffer, decoded_bytes, OUTPUT_LEN_BYTES);
	}
	void sync() {
		size_t lim = in.readable() / (cr[1] * 8 * OUTPUT_LEN_BYTES);
		size_t output_write = cr[0] * lim * OUTPUT_LEN_BYTES;

		if (cr[0] * lim <= 1)
    		return;

		u8* pin = reinterpret_cast<u8*>(in.rd());
		float BER[4] = {};
		u8* dec_hyp = new u8[output_write];
		u8* enc_hyp = new u8[2 * 8 * output_write];
		for (int i = 0; i < punc_phase_num; ++i) {
        	decoder_reset();
			uint32_t idx_out = 0;
			uint32_t idx_in = 2 * i;

			int iter = cr[0] * lim;

			while (iter > 1) {
				idx_in += puncturer(pin + idx_in, input_buffer, punc_offset);
            	decode(dec_hyp + idx_out);
				idx_out += OUTPUT_LEN_BYTES;
				iter--;
			}
			// Re-encode decoded hypothesis
			encode(enc_hyp, dec_hyp + (TRACEBACK >> 3), idx_out - (TRACEBACK >> 3), 0);

			// Compare hypothesis against received data
			uint32_t ber = 0;
			idx_in = 2 * i;
			for (int j = 0; j < 2 * 8 * idx_out - 4 * TRACEBACK; j += 2 * cr[0]) {
				for (int k = 0; k < 2 * cr[0]; ++k) {
					if (patterns[crate][k]) {
						ber += enc_hyp[j + k] != (u8)(pin[idx_in++] > 63);
					}
				}
			}
        	BER[i] = (float)ber / (idx_in - 2 * i);
    	}

		// Find puncturing phase with minimum BER
		uint8_t argmin = 0;
		for (int i = 1; i < punc_phase_num; ++i) {
			if (BER[i] < BER[argmin])
				argmin = i;
		}

		// Align input stream to detected puncturing phase
		if (BER[argmin] < 0.01f) {
			in.read(2 * argmin);
			sync_flag = 1;
		} else {
			in.read(in.readable()); // Discard failed input; release pipe space.
		}

		decoder_reset();

		delete[] dec_hyp;
		delete[] enc_hyp;
	}
		
public:
	viterbi(scheduler* sch, pipebuf<u8>& _in,
		pipebuf<u8>& _out, code_rate r) :runnable(sch, "VITERBI_S"),
		out(_out), in(_in), crate(r)
	{
		switch (r) {
		case FEC12:
			puncturer = punc_1_2;
			cr[0] = 1; cr[1] = 2;
			punc_phase_num = 1;
			break;
		case FEC23:
			puncturer = punc_2_3;
			cr[0] = 2; cr[1] = 3;
			punc_phase_num = 3;
			break;
		case FEC34:
			puncturer = punc_3_4;
			cr[0] = 3; cr[1] = 4;
			punc_phase_num = 2;
			break;
		case FEC56:
			puncturer = punc_5_6;
			cr[0] = 5; cr[1] = 6;
			punc_phase_num = 3;
			break;
		case FEC78:
			puncturer = punc_7_8;
			cr[0] = 7; cr[1] = 8;
			punc_phase_num = 4;
			break;
		default:
			fatal("rate not supported");
			break;
		}
		out.buf.min_write = cr[0] * OUTPUT_LEN_BYTES;
	}
	void run() {
		if (!sync_flag) {
			sync();
			if (!sync_flag){
				return;
			}
		}

		size_t out_lim = out.writable() / (cr[0] * OUTPUT_LEN_BYTES);
		size_t in_lim = in.readable() / (cr[1] * 8 * OUTPUT_LEN_BYTES);

		size_t lim = std::min(out_lim, in_lim);
		size_t iter = cr[0] * lim;
		size_t input_read = cr[1] * lim * 8 * OUTPUT_LEN_BYTES;

		u8* pin = reinterpret_cast<u8*>(in.rd());
		while (iter > 0) {
			int r = puncturer(pin, input_buffer, punc_offset);
			decode(out.wr());

			if (skip_flag){
    			std::memmove(out.wr(), out.wr() + TRACEBACK/8, OUTPUT_LEN_BYTES - TRACEBACK/8);
				out.written(OUTPUT_LEN_BYTES - TRACEBACK/8);
				skip_flag = 0;
			}
			
			else
				out.written(OUTPUT_LEN_BYTES);
			pin += r;
			iter--;
		}
		in.read(input_read);

		uint32_t min_metric = state[0];
		for (int i = 1; i < 64; ++i)
			if (state[i] < min_metric)
				min_metric = state[i];

		for (int i = 0; i < 64; ++i)
			state[i] -= min_metric;
	}
};

class conv_encoder : public runnable {
private:
    pipereader<u8> in;
    pipewriter<u8> out;

    unsigned numerator = 0;
    unsigned denominator = 0;
    const bool* pattern = nullptr;
    u8 enc_state = 0;

public:
    conv_encoder(scheduler* sch, pipebuf<u8>& input, pipebuf<u8>& output, code_rate rate):
	runnable(sch, "CONV_ENCODER"), in(input), out(output){
        switch (rate) {
        case FEC12: numerator = 1; denominator = 2; break;
        case FEC23: numerator = 2; denominator = 3; break;
        case FEC34: numerator = 3; denominator = 4; break;
        case FEC56: numerator = 5; denominator = 6; break;
        case FEC78: numerator = 7; denominator = 8; break;
        default:
            fatal("encoder rate not supported");
            return;
        }
        pattern = patterns[rate];
        out.buf.min_write = denominator;
    }
    void run() override {
        size_t groups = std::min(in.readable() / numerator, out.writable() / denominator);

        if (groups == 0)
            return;

        const u8* src = in.rd();
        u8* dst = out.wr();

        const unsigned polys[] = {POLYA, POLYB};
        unsigned phase = 0;
        unsigned packed_bits = 0;
        u8 packed = 0;

        for (size_t i = 0; i < groups * numerator; ++i) {
            u8 value = *src++;

            for (int bit = 7; bit >= 0; --bit) {
                enc_state = static_cast<u8>((enc_state << 1) | ((value >> bit) & 1));
                for (unsigned poly : polys) {
                    if (pattern[phase]) {
                        packed = static_cast<u8>((packed << 1) | d_Partab[enc_state & poly]);
                        if (++packed_bits == 8) {
                            *dst++ = packed;
                            packed = 0;
                            packed_bits = 0;
                        }
                    }
                    if (++phase == 2 * numerator)
                        phase = 0;
                }
            }
        }
        in.read(groups * numerator);
        out.written(groups * denominator);
    }
};
#endif
