#ifndef SOUNDLINK_DATALINK_H
#define SOUNDLINK_DATALINK_H


#include <fstream>
#include <filesystem>
#include <cstdint>
#include <vector>
#include <string>
#include <stdexcept>
#include <cstring>
#include <bitset>
#include "framework.h"

namespace soundlink{
constexpr uint8_t packet_marker = 0xD3;
// enums

// create control packets
enum class ctrl_type:uint8_t {
    hello, hello_ack, probe_report, config,
    config_ack, ack, timing_adj,
    close, close_ack
};
// packet type
enum class packet_type:uint8_t {
    data = 0,
    ctrl = 1
};
//loading status
enum class load_status:uint8_t {
    loaded = 0,
    failed = 1,
    complete = 2,
};
// storing status
enum class store_status:uint8_t {
    stored = 0,
    failed = 1,
    complete = 2,
};
enum class parse_status {
    ready,
    incomplete,
    no_packet
};

// packet layouts
struct TxFrame {
    bool ack = false;
    uint32_t offset;          // Offset into the owned message buffer
    uint16_t sequence;
    uint8_t length;
    uint8_t tx_count = 0;
};
struct file_info {
    std::string file_name;
    uint32_t file_size;
    uint8_t segment_size;
};
struct channel_info {
    uint8_t fc;
    uint8_t code_rate;
    uint8_t bandwidth;
};
using chinfo = std::vector<channel_info>;

struct ack {
    uint16_t base;
    std::vector<uint8_t> bitmap;
};

struct ctrl_packet {
    uint16_t sequence;
    ctrl_type type;
    std::vector<uint8_t> fields;
    uint32_t tx_count = 0;
};
struct ctrl_factory {
    static ctrl_packet hello(const file_info& info);
    static ctrl_packet hello_ack(uint16_t sequence);
    static ctrl_packet probe_report(const std::vector<uint8_t>& snr);
    static ctrl_packet config(const chinfo& data_channels, const chinfo& ctrl_channels);
    static ctrl_packet config_ack(uint16_t sequence);
    static ctrl_packet timing_adjustment(int16_t samples);
    static ctrl_packet close();
    static ctrl_packet close_ack(uint16_t sequence);
};

// Fields: [file size:4B, big-endian][segment size:1B][filename bytes].
inline ctrl_packet ctrl_factory::hello(const file_info& info) {
    if (info.file_name.size() > 249)
        throw std::runtime_error("Filename exceeds hello packet capacity");

    ctrl_packet packet{};
    packet.type = ctrl_type::hello;
    packet.fields = {uint8_t(info.file_size >> 24),
        uint8_t(info.file_size >> 16),
        uint8_t(info.file_size >> 8),
        uint8_t(info.file_size),
        info.segment_size
    };
    packet.fields.insert(packet.fields.end(),info.file_name.begin(), info.file_name.end());
    return packet;
}
inline file_info read_hello(const ctrl_packet& packet) {
    if (packet.type != ctrl_type::hello || packet.fields.size() < 5)
        throw std::runtime_error("Invalid hello packet");

    const auto& f = packet.fields;
    file_info info;
    info.file_size = (uint32_t(f[0]) << 24)
                   | (uint32_t(f[1]) << 16)
                   | (uint32_t(f[2]) << 8)
                   | uint32_t(f[3]);
    info.segment_size = f[4];
    info.file_name.assign(f.begin() + 5, f.end());
    return info;
}
inline ctrl_packet ctrl_factory::hello_ack(uint16_t sequence) {
    ctrl_packet packet{};
    packet.type = ctrl_type::hello_ack;
    packet.fields = {
        uint8_t(sequence >> 8),
        uint8_t(sequence)
    };
    return packet;
}

// One byte per channel, in probing order: SNR in dB = value * 30 / 255.
inline ctrl_packet ctrl_factory::probe_report(const std::vector<uint8_t>& snr) {
    ctrl_packet packet{};
    packet.type = ctrl_type::probe_report;
    packet.fields = snr;
    return packet;
}
// Fields: [data count:1B][ctrl count:1B]
// Followed by data channels, then control channels.
// Each channel: [fc:1B][code rate:1B][bandwidth:1B]
inline ctrl_packet ctrl_factory::config(
    const chinfo& data_channels, const chinfo& ctrl_channels) {

    if (data_channels.size() + ctrl_channels.size() > 84)
        throw std::runtime_error("Too many channels for config packet");

    ctrl_packet packet{};
    packet.type = ctrl_type::config;
    packet.fields = {
        uint8_t(data_channels.size()),
        uint8_t(ctrl_channels.size())
    };

    for (const auto& ch : data_channels)
        packet.fields.insert(packet.fields.end(), {ch.fc, ch.code_rate, ch.bandwidth});

    for (const auto& ch : ctrl_channels)
        packet.fields.insert(packet.fields.end(), {ch.fc, ch.code_rate, ch.bandwidth});
    return packet;
}
inline void read_config(const ctrl_packet& packet, chinfo& data_channels, chinfo& ctrl_channels) {
    const auto& f = packet.fields;

    if (packet.type != ctrl_type::config || f.size() < 2)
        throw std::runtime_error("Invalid config packet");

    size_t data_count = f[0];
    size_t ctrl_count = f[1];

    if (f.size() != 2 + 3 * (data_count + ctrl_count))
        throw std::runtime_error("Invalid config length");

    data_channels.clear();
    ctrl_channels.clear();

    size_t offset = 2;
    for (size_t i = 0; i < data_count; ++i) {
        data_channels.push_back({f[offset], f[offset + 1], f[offset + 2]});
        offset += 3;
    }
    for (size_t i = 0; i < ctrl_count; ++i) {
        ctrl_channels.push_back({f[offset], f[offset + 1], f[offset + 2]});
        offset += 3;
    }
}
inline ctrl_packet ctrl_factory::config_ack(uint16_t sequence) {
    ctrl_packet packet{};
    packet.type = ctrl_type::config_ack;
    packet.fields = {
        uint8_t(sequence >> 8),
        uint8_t(sequence)
    };
    return packet;
}
inline ctrl_packet ctrl_factory::timing_adjustment(int16_t samples) {
    uint16_t value = static_cast<uint16_t>(samples);

    ctrl_packet packet{};
    packet.type = ctrl_type::timing_adj;
    packet.fields = {
        uint8_t(value >> 8),
        uint8_t(value)
    };
    return packet;
}
inline ctrl_packet ctrl_factory::close() {
    ctrl_packet packet{};
    packet.type = ctrl_type::close;
    return packet;
}
inline ctrl_packet ctrl_factory::close_ack(uint16_t sequence) {
    ctrl_packet packet{};
    packet.type = ctrl_type::close_ack;
    packet.fields = {
        uint8_t(sequence >> 8),
        uint8_t(sequence)
    };
    return packet;
}
struct TxMessage {
    std::vector<uint8_t> payload;
    TxMessage(const std::filesystem::path& path, size_t max_len):
        cursor(0),
        acked(0){
        size_t max_size = 10 * 1024 * 1024; // 10 MiB
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        if (!file)
            throw std::runtime_error("Cannot open file");

        const auto file_size = file.tellg();
        if (file_size < 0)
            throw std::runtime_error("Cannot determine file size");
        size_t size = static_cast<size_t>(file_size);
        
        if (size > max_size)
            throw std::runtime_error("File exceeds size limit");

        payload.resize(size);
        file.seekg(0, std::ios::beg);

        if (!payload.empty())
            file.read(reinterpret_cast<char*>(payload.data()),
            static_cast<std::streamsize>(payload.size()));

        if (!file) 
            throw std::runtime_error("Cannot read entire file");
        
        uint32_t offset = 0;
        while(size != 0){
            TxFrame frame;
            size_t len = size < max_len?size:max_len;
            frame.offset = offset;
            frame.sequence = static_cast<uint16_t>(frames.size());
            frame.length = static_cast<uint8_t>(len);
            frames.push_back(frame);
            size -= len;
            offset += len;
        }
    }
    bool completed(){
        return acked == frames.size();
    }
    void acknowledge(size_t sequence) {
        if (sequence  >= frames.size()) return;
        if (!frames[sequence].ack){
            frames[sequence].ack = true;
            acked++;
        }   
    }
    TxFrame& next() {
        if (completed())
            throw std::runtime_error("called while transfer finished");
        while (frames[cursor].ack){
            cursor++;
            if (cursor == frames.size()) cursor = 0;
        }
        TxFrame& frame = frames[cursor];
        cursor++;
        if (cursor == frames.size()) cursor = 0;
        return frame;
    }
private:
    std::vector<TxFrame> frames;
    size_t cursor;
    size_t acked;
};
struct RxMessage {
    std::vector<uint8_t> payload;
    std::vector<bool> received;
    size_t segment_size;
    size_t received_count = 0;
    std::filesystem::path path;

    RxMessage(std::filesystem::path _path, size_t file_size, size_t segment_size)
        : segment_size(segment_size),
        path(_path){
        if (file_size > 10u * 1024 * 1024)
            throw std::invalid_argument("File exceeds 10 MiB");

        if (segment_size == 0)
            throw std::invalid_argument("Invalid segment size");

        const size_t segments = file_size /
         segment_size + (file_size % segment_size != 0);

        if (segments > 65536)
            throw std::invalid_argument("Too many segments");

        payload.resize(file_size);
        received.resize(segments, false);
    }
    // True for valid segments, including duplicates so they can be ACKed again.
    void store(size_t sequence, const void* input, size_t size){
        if (sequence >= received.size()) return;
        if (received[sequence]) return;
        const size_t offset = sequence * segment_size;
    
        std::memcpy(payload.data() + offset, input, size);
        received[sequence] = true;
        ++received_count;
    }
    void flush_to_file(){
        if (received_count != received.size())
            throw std::runtime_error("Cannot save incomplete message");
        std::ofstream file(path, std::ios::binary | std::ios::trunc);
        if (!file)
            throw std::runtime_error("Cannot open output file");

        if (!payload.empty())
            file.write(reinterpret_cast<const char*>(payload.data()),
                       static_cast<std::streamsize>(payload.size()));
        file.close();
        if (!file)
            throw std::runtime_error("Cannot write output file");
    }
    bool complete() const {
        return received_count == received.size();
    }
};

struct TxctrlQueue {
    TxctrlQueue(size_t m):maxqueue(m), sequence(0){};
    // New messages queued
    bool push(ctrl_packet packet) {
        if (packet.type == ctrl_type::ack){
            return push_ack(packet.sequence);
        }
        if (packets.size() >= maxqueue) return 0;
        packet.sequence = sequence;
        sequence++;
        packets.push_back(std::move(packet));
        return 1;
    }
    bool push_ack(uint16_t segment){
        uint16_t base = segment & 0xFFE0u;
        uint32_t bit = uint32_t{1} << (segment & 31u);
        for (auto& packet : packets) {
            if (packet.type != ctrl_type::ack)
                continue;
            uint16_t stored_base = (uint16_t(packet.fields[0]) << 8) | packet.fields[1];

            if (stored_base != base)
                continue;

            uint32_t bitmap = 0;
            for (unsigned i = 0; i < 4; ++i)
                bitmap = (bitmap << 8) | packet.fields[2 + i];

            if (bitmap & bit)
                return true; // Already represented.

            bitmap |= bit;

            for (unsigned i = 0; i < 4; ++i)
                packet.fields[2 + i] =
                    uint8_t(bitmap >> (24 - 8 * i));

            packet.sequence = sequence++;
            packet.tx_count = 0;
            return true;
        }

        ctrl_packet packet{};
        packet.type = ctrl_type::ack;
        packet.fields.resize(6);
        packet.fields[0] = uint8_t(base >> 8);
        packet.fields[1] = uint8_t(base);

        for (unsigned i = 0; i < 4; ++i)
            packet.fields[2 + i] = uint8_t(bit >> (24 - 8 * i));
        if (packets.size() >= maxqueue) {
            if (!remove_ack())
                return false;
        }
        packet.sequence = sequence;
        sequence++;
        packets.push_back(std::move(packet));
        return true;
    }
    bool empty() const {
        return packets.empty();
    }

    ctrl_packet& next() {
        auto* selected = &packets[0];

        for (auto& packet : packets) {
            if (packet.tx_count < selected->tx_count)
                selected = &packet;
        }
        return *selected;
    }
    void remove(uint16_t sequence) {
        for (size_t i = 0; i < packets.size(); ++i) {
            if (packets[i].sequence != sequence)
                continue;
            packets.erase(packets.begin() + i);
            return;
        }
    }
    bool remove_ack() {
        auto selected = packets.end();
        for (auto it = packets.begin(); it != packets.end(); ++it) {
            if (it->type != ctrl_type::ack || it->tx_count == 0)
                continue;
            if (selected == packets.end() ||
                it->tx_count > selected->tx_count)
                selected = it;
        }
        if (selected == packets.end())
            return false;
        packets.erase(selected);
        return true;
    }
private:
    std::vector<ctrl_packet> packets;
    size_t maxqueue;
    uint16_t sequence;
};

struct rxCtrlQueue {
    rxCtrlQueue(size_t m) : maxqueue(m) {}

    bool push(ctrl_packet packet) {
        if(received[packet.sequence]) return true;
        if (packets.size() >= maxqueue)
            return false;
        
        received[packet.sequence] = true;
        packets.push_back(std::move(packet));
        return true;
    }

    bool empty() const {
        return packets.empty();
    }

    // Caller must check empty() first.
    ctrl_packet& next() {
        return packets.front();
    }

    void pop() {
        if (!packets.empty())
            packets.erase(packets.begin());
    }

private:
    std::vector<ctrl_packet> packets;
    size_t maxqueue;
    std::bitset<65536> received;
};
// CRC GEN
inline uint32_t crc32c(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit) {
            if (crc & 1u)
                crc = (crc >> 1) ^ 0x82F63B78u;
            else
                crc >>= 1;
        }
    }
    return crc ^ 0xFFFFFFFFu;
}
// Packet: [Marker:1B][Type:1B][segment:2B][Length:1B][Payload:0–255B][CRC-32C:4B]
// Maximum total: 264 bytes; Length counts payload bytes; CRC covers Type through Payload.
inline size_t link_write_data(TxMessage& m, u8* dest, size_t writable){
    size_t written = 0;
    while(1){
        if(m.completed()) return written;
        TxFrame& frame = m.next();
        size_t required = 9 + (size_t)frame.length;
        if(writable < required) break;
        dest[0] = packet_marker;
        dest[1] = (u8)packet_type::data;
        dest[2] = (u8)(frame.sequence >> 8 & 0xFF);
        dest[3] = (u8)(frame.sequence & 0xFF);
        dest[4] = (u8)(frame.length);
        std::memcpy(dest+5, m.payload.data() + frame.offset, frame.length);
        uint32_t crc = crc32c((const uint8_t*)dest+1, 4 + (size_t)frame.length);
        for (unsigned i = 0; i < 4; ++i)
            dest[5 + frame.length + i] = (u8)((crc >> (24 - 8 * i)) & 0xFFu);
            
        frame.tx_count++;
        writable -= required;
        dest += required;
        written += required;
    }
    return written;
}
// Packet: [Marker:1B][Type:1B][segment:2B][Length:1B][subType:1B][Payload:0–254B][CRC-32C:4B]
// Maximum total: 264 bytes; Length counts subtype + fields; CRC covers Type through fields.
inline size_t link_write_ctrl(ctrl_packet& packet, u8* dest, size_t writable){
    size_t payload_size = packet.fields.size() + 1; // plus 1 for subtype
    if(payload_size >= 256) throw std::runtime_error("field size issue!");
    if(writable < 9 + payload_size) return 0;
    dest[0] = packet_marker;
    dest[1] = (u8)packet_type::ctrl;
    dest[2] = (u8)(packet.sequence >> 8 & 0xFF);
    dest[3] = (u8)(packet.sequence & 0xFF);
    dest[4] = (u8)(payload_size);
    dest[5] = (u8)(packet.type);
    if (payload_size>1)
        std::memcpy(dest + 6, packet.fields.data(), packet.fields.size());

    uint32_t crc = crc32c((const uint8_t*)dest + 1, 4 + payload_size);

    for (unsigned i = 0; i < 4; ++i)
        dest[5 + payload_size + i] = (u8)((crc >> (24 - 8 * i)) & 0xFFu);
    packet.tx_count++;
    return 9 + payload_size;
}
// Packet: [Marker:1B][Type:1B][segment:2B][Length:1B][subType:1B][Payload:0–254B][CRC-32C:4B]
inline parse_status parser(const u8* in, size_t& offset, size_t readable){
    size_t cursor = 0;
    while(cursor + 9 < readable){
        if(in[cursor++] != packet_marker) continue;
        if((uint8_t)in[cursor] != uint8_t(packet_type::data)
        && (uint8_t)in[cursor] != uint8_t(packet_type::ctrl)) continue;
        if (in[cursor + 3] == 0) continue;

        size_t packet_len = 9 + (size_t)in[cursor + 3];
        if (cursor + packet_len - 1 > readable){
            offset = cursor - 1;
            return parse_status::incomplete;
        }
        uint32_t crc = crc32c((const uint8_t*)(in + cursor), packet_len - 5);
        bool check = true;
        for(int i = 0; i < 4; ++i){
            u8 c = u8(crc >> (24 - 8 * i));;
            if (in[cursor + packet_len - 5 + i] != c) check = false;
        }
        if(!check) continue;
        offset = cursor - 1;
        return parse_status::ready;
    }
    offset = cursor;
    return parse_status::no_packet;

}
struct mtx_loader:runnable{
    mtx_loader(scheduler* sch, TxMessage& _m, pipebuf<u8>& _out)
        : runnable(sch, "MTX"), out(_out, 264), m(_m) {}
    void run() override{
        size_t available = out.writable();
        size_t w = link_write_data(m, out.wr(), available);
        out.written(w);
    }
private:
    pipewriter<u8> out;
    TxMessage& m;
};
struct ctrl_loader:runnable{
    ctrl_loader(scheduler* sch, TxctrlQueue& _tctrl, pipebuf<u8>& _out)
    : runnable(sch, "CTRLTX"), out(_out, 264), tctrl(_tctrl){}
    void run() override {
        size_t written = 0;
        while (written < minwrite) {
            size_t available = out.writable();
            if (available == 0) break;
            u8* wr = out.wr();
            size_t count;
            if (!tctrl.empty()) {
                count = link_write_ctrl(tctrl.next(), wr, available);
                if (count == 0) break;
            } else {
                size_t needed = minwrite - written;
                count = needed < available ? needed : available;
                std::memset(wr, 0, count);
            }
            out.written(count);
            written += count;
        }
    }
private:
    static constexpr size_t minwrite = 256;
    pipewriter<u8> out;
    TxctrlQueue& tctrl;
};

struct mrx_store:runnable{
    mrx_store(scheduler* sch, pipebuf<u8>& _in, RxMessage& _rx, TxctrlQueue& _tctrl)
    :runnable(sch, "MRX"), in(_in), rx(_rx), tctrl(_tctrl){}
    void run() override{
        size_t offset;
        parse_status status;
        while(true){
            status = parser(in.rd(), offset, in.readable());
            if (status != parse_status::ready){
                in.read(offset);
                break;
            }
            u8* rd = in.rd() + offset;
            if (rd[1] != (u8)packet_type::data){
                in.read(1);
                continue;
            }
            uint16_t segment = uint16_t(rd[2] << 8) | uint16_t(rd[3]);
            if(segment >= rx.received.size()){
                in.read(1);
                continue;
            }
            size_t length = size_t(rd[4]);
            if (length == 0 || length > rx.segment_size){
                in.read(1);
                continue;
            }
            rx.store(segment, rd + 5, length);
            tctrl.push_ack(segment);
            in.read(offset + 9 + length);
        }
    }
private:
    pipereader<u8> in;
    RxMessage& rx;
    TxctrlQueue& tctrl;
};
struct ctrl_store : runnable {
    ctrl_store(scheduler* sch, pipebuf<u8>& _in, rxCtrlQueue& _rctrl)
        :runnable(sch, "CTRLRX"), in(_in), rctrl(_rctrl){}

    void run() override {
        while (true) {
            size_t offset;
            auto status = parser(in.rd(), offset, in.readable());

            if (status != parse_status::ready) {
                in.read(offset);
                return;
            }

            u8* rd = in.rd() + offset;

            if (rd[1] != u8(packet_type::ctrl)) {
                in.read(1);
                continue;
            }
            size_t length = rd[4];
            if (length == 0 || rd[5] > u8(ctrl_type::close_ack)) {
                in.read(1);
                continue;
            }

            ctrl_packet packet{};
            packet.sequence = (uint16_t(rd[2]) << 8) | rd[3];
            packet.type = static_cast<ctrl_type>(rd[5]);
            packet.fields.assign(rd + 6, rd + 5 + length);

            if (!rctrl.push(std::move(packet))) {
                in.read(offset); // Discard preceding junk; retain packet.
                return;
            }

            in.read(offset + 9 + length);
        }
    }

private:
    pipereader<u8> in;
    rxCtrlQueue& rctrl;
};

}
#endif
