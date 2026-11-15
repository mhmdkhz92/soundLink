%module soundlink

%{
#define MINIAUDIO_IMPLEMENTATION
#include "src/xchain.h"
%}

%include <std_string.i>
%include <std_vector.i>
%include <stdint.i>
%include <exception.i>

%exception {
    try {
        $action
    } catch (const std::exception& error) {
        SWIG_exception(SWIG_RuntimeError, error.what());
    }
}

%rename(Sender) soundlink::sender;
%rename(Receiver) soundlink::receiver;
%rename(ChannelInfo) soundlink::channel_info;
%rename(LinkConfig) soundlink::link_cfg;
%rename(Report) soundlink::report;
%feature("flatnested") soundlink::report::channel;
%rename(ChannelReport) soundlink::report::channel;
%nodefaultctor soundlink::sender;
%nodefaultctor soundlink::receiver;
%nodefaultctor soundlink::channel_info;
%nodefaultctor soundlink::link_cfg;
%nodefaultctor soundlink::report;
%nodefaultctor soundlink::report::channel;
%immutable;

%traits_swigtype(soundlink::report::channel);
%traits_swigtype(soundlink::link_cfg);

// Use SWIG's value conversions for the fields of a report snapshot.
%typemap(out, fragment="StdVectorTraits", fragment=SWIG_Traits_frag(soundlink::report::channel)) std::vector<soundlink::report::channel> * {
    $result = swig::from(*$1);
}
%typemap(out, fragment=SWIG_Traits_frag(soundlink::link_cfg)) soundlink::link_cfg * {
    $result = swig::from(*$1);
}

namespace soundlink {
    enum BW { BW1_4, BW3, BW5, BW10 };
    enum modulation { BPSK, QPSK, QAM16, QAM64 };
    enum code_rate { FEC12, FEC23, FEC34, FEC56, FEC78 };

    struct channel_info {
        uint8_t fc;
        uint8_t code_rate;
        uint8_t bandwidth;
        uint8_t mod;
    };
    struct link_cfg {
        int fc;
        BW bw;
        float bandwidth;
        code_rate cr;
        modulation mod;
    };

    struct report {
        struct channel {
            const char* name;
            link_cfg config;
            float snr_db;
            bool snr_valid;
            size_t received_bytes;
        };

        std::string state;
        std::string file_name;
        size_t total_bytes;
        size_t bytes_done;
        size_t tx_queued;
        size_t rx_queued;
        std::vector<channel> tx;
        std::vector<channel> rx;
    };
}

%template(ChannelVector) std::vector<soundlink::channel_info>;

%extend soundlink::channel_info {
    channel_info(uint8_t fc, uint8_t code_rate, uint8_t bandwidth, uint8_t mod) {
        return new soundlink::channel_info{fc, code_rate, bandwidth, mod};
    }
}

// SWIG sees only this API. The generated C++ includes the real definitions.
namespace soundlink {
    struct sender {
        ~sender();
        void start();
        void step();
        bool is_finished() const;
        report get_report();
    };

    struct receiver {
        ~receiver();
        void start();
        void step();
        bool is_finished() const;
        report get_report();
    };
}

%extend soundlink::sender {
    sender(const std::string& path) {
        return new soundlink::sender(std::filesystem::u8path(path));
    }
}

%extend soundlink::receiver {
    receiver(const std::string& path) {
        return new soundlink::receiver(std::filesystem::u8path(path));
    }

    void set_tr_channels(std::vector<soundlink::channel_info> data,
                         std::vector<soundlink::channel_info> ctrl) {
        $self->set_tr_channels(&data, &ctrl);
    }
}
