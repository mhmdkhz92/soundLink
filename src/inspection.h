#ifndef SOUNDLINK_INSPECTION_H
#define SOUNDLINK_INSPECTION_H

#include "xchain.h"
#include <chrono>
#include <deque>
#include <optional>
#include <cmath>
#include <unordered_map>
#include <array>
#include <atomic>
#include <mutex>

namespace soundlink {

using inspection_snapshot = std::vector<module_snapshot>;

namespace inspection_detail {

template<typename State>
std::string engine_state(State state, bool finished) {
    if (finished) return "finished";
    switch (state) {
    case State::startup: return "not started";
    case State::handshake: return "handshake";
    case State::transfer: return "transfer";
    case State::close: return "closing";
    }
    return "unknown";
}

inline void append_report(module_snapshot& target, module_snapshot source, const std::string& prefix) {
    for (auto& field : source.fields) {
        field.id = prefix + field.id;
        target.fields.push_back(std::move(field));
    }
}

inline const inspect_value* find_value(const module_snapshot& snapshot, const std::string& id) {
    for (const auto& field : snapshot.fields)
        if (field.id == id) return &field.value;
    return nullptr;
}

}

inline module_snapshot inspect(const link_cfg& cfg) {
    std::string modulation_name, rate;
    switch (cfg.mod) {
    case BPSK: modulation_name = "BPSK"; break;
    case QPSK: modulation_name = "QPSK"; break;
    case QAM16: modulation_name = "16-QAM"; break;
    case QAM64: modulation_name = "64-QAM"; break;
    default: modulation_name = "unknown"; break;
    }
    switch (cfg.cr) {
    case FEC12: rate = "1/2"; break;
    case FEC23: rate = "2/3"; break;
    case FEC34: rate = "3/4"; break;
    case FEC56: rate = "5/6"; break;
    case FEC78: rate = "7/8"; break;
    default: rate = "unknown"; break;
    }
    return {"link", "Channel configuration", {
        {"center_frequency", "Center frequency", "kHz", std::int64_t(cfg.fc)},
        {"bandwidth", "Bandwidth", "Hz", double(cfg.bandwidth)},
        {"modulation", "Modulation", "", modulation_name},
        {"code_rate", "Code rate", "", rate}
    }};
}

inline module_snapshot txdata_ch::inspect() const {
    auto snapshot = soundlink::inspect(cfg);
    snapshot.id = "data";
    snapshot.label = "Transmit data channel";
    return snapshot;
}

inline module_snapshot txctrl_ch::inspect() const {
    auto snapshot = soundlink::inspect(cfg);
    snapshot.id = "control";
    snapshot.label = "Transmit control channel";
    return snapshot;
}

inline module_snapshot equalizer::inspect(modulation mod) const {
    inspect_value snr = std::string("unavailable");
    double kurtosis = 0.0;
    switch (mod) {
    case BPSK: case QPSK: kurtosis = 1.0; break;
    case QAM16: kurtosis = 1.32; break;
    case QAM64: kurtosis = 1.38095238; break;
    default: return {"equalizer", "Equalizer", {{"snr", "Estimated SNR", "dB", snr}}};
    }

    double signal_power = 0.0, noise_power = 0.0;
    bool valid = noise.acc > 0 && noise.m2.size() == nRB && noise.m4.size() == nRB && nRB > 0;
    for (size_t i = 0; valid && i < nRB; ++i) {
        const double m2 = noise.m2[i], m4 = noise.m4[i];
        if (!std::isfinite(m2) || !std::isfinite(m4) || m2 <= 0.0 || m4 < 0.0) {
            valid = false;
            break;
        }
        const double signal = std::sqrt(std::max(0.0, (2.0 * m2 * m2 - m4) / (2.0 - kurtosis)));
        const double background = m2 - signal;
        if (signal <= 0.0 || background <= 0.0) {
            valid = false;
            break;
        }
        signal_power += signal;
        noise_power += background;
    }
    if (valid) snr = 10.0 * std::log10(signal_power / noise_power);
    return {"equalizer", "Equalizer", {{"snr", "Estimated SNR", "dB", snr}}};
}

inline module_snapshot slot_demapper::inspect() const {
    if (state != mode::tracking)
        return {"slot_demapper", "Slot demapper", {{"snr", "Estimated SNR", "dB", std::string("unavailable")}}};
    auto snapshot = eq.inspect(cfg.mod);
    snapshot.id = "slot_demapper";
    snapshot.label = "Slot demapper";
    return snapshot;
}

inline module_snapshot mrx_store::inspect() const {
    return {"receive_payload", "Received payload", {
        {"accepted_payload_bytes", "Accepted payload bytes", "bytes", accepted_payload_bytes}
    }};
}

inline module_snapshot rxdata_ch::inspect() const {
    auto snapshot = soundlink::inspect(cfg);
    snapshot.id = "data";
    snapshot.label = "Receive data channel";
    inspection_detail::append_report(snapshot, mapper.inspect(), "");
    snapshot.fields.push_back({"constellation_id", "Constellation channel", "", std::uint64_t(reinterpret_cast<std::uintptr_t>(&mapper))});
    inspection_detail::append_report(snapshot, sink.inspect(), "");
    snapshot.fields.push_back({"rate_identity", "Rate identity", "",
        std::to_string(reinterpret_cast<std::uintptr_t>(this)) + ":" + std::to_string(stamp.origin)});
    return snapshot;
}

inline module_snapshot rxctrl_ch::inspect() const {
    auto snapshot = soundlink::inspect(cfg);
    snapshot.id = "control";
    snapshot.label = "Receive control channel";
    inspection_detail::append_report(snapshot, mapper.inspect(), "");
    snapshot.fields.push_back({"constellation_id", "Constellation channel", "", std::uint64_t(reinterpret_cast<std::uintptr_t>(&mapper))});
    return snapshot;
}

namespace inspection_detail {

template<typename Channel>
void append_channels(module_snapshot& engine, const std::vector<Channel*>& channels, const std::string& direction) {
    for (size_t i = 0; i < channels.size(); ++i) {
        const auto* source = dynamic_cast<const inspectable*>(channels[i]);
        const std::string prefix = "channels." + direction + "." + std::to_string(i);
        const std::string label = direction + " CH" + std::to_string(i + 1);
        auto snapshot = source ? source->inspect() : module_snapshot{"unknown", "Unknown channel", {}};
        engine.fields.push_back({prefix + ".kind", label + " kind", "", snapshot.id});
        for (auto& field : snapshot.fields) {
            field.id = prefix + "." + field.id;
            field.label = label + ": " + field.label;
            engine.fields.push_back(std::move(field));
        }
    }
}

}

inline module_snapshot sender::inspect() const {
    module_snapshot snapshot{"sender", "Sender", {
        {"state", "State", "", inspection_detail::engine_state(current, finished)}
    }};
    snapshot.fields.reserve(11 + 5 * tx.size() + 9 * rx.size());
    inspection_detail::append_channels(snapshot, tx, "tx");
    inspection_detail::append_channels(snapshot, rx, "rx");
    inspection_detail::append_report(snapshot, msg.inspect(), "transfer.");
    inspection_detail::append_report(snapshot, txctrl.inspect(), "control.tx.");
    inspection_detail::append_report(snapshot, rxctrl.inspect(), "control.rx.");
    return snapshot;
}

inline module_snapshot receiver::inspect() const {
    module_snapshot snapshot{"receiver", "Receiver", {
        {"state", "State", "", inspection_detail::engine_state(current, finished)}
    }};
    snapshot.fields.reserve(11 + 5 * tx.size() + 9 * rx.size());
    inspection_detail::append_channels(snapshot, tx, "tx");
    inspection_detail::append_channels(snapshot, rx, "rx");
    if (msg) inspection_detail::append_report(snapshot, msg->inspect(), "transfer.");
    inspection_detail::append_report(snapshot, txctrl.inspect(), "control.tx.");
    inspection_detail::append_report(snapshot, rxctrl.inspect(), "control.rx.");
    return snapshot;
}

inline module_snapshot TxMessage::inspect() const {
    std::uint64_t acknowledged_bytes = 0;
    if (!frames.empty()) {
        const auto& last = frames.back();
        acknowledged_bytes = std::uint64_t(acked - size_t(last.ack)) * frames.front().length;
        if (last.ack) acknowledged_bytes += last.length;
    }

    const double progress = payload.empty() ? 1.0 : double(acknowledged_bytes) / double(payload.size());
    return {"sender.message", "Outgoing file", {
        {"file_name", "File name", "", metadata.file_name},
        {"file_size", "File size", "bytes", std::uint64_t(payload.size())},
        {"acknowledged_bytes", "Acknowledged bytes", "bytes", acknowledged_bytes},
        {"progress", "Progress", "", progress},
        {"completed", "Completed", "", acked == frames.size()}
    }};
}

inline module_snapshot RxMessage::inspect() const {
    std::uint64_t received_bytes = 0;
    if (!received.empty()) {
        const bool last_received = received.back();
        received_bytes = std::uint64_t(received_count - size_t(last_received)) * segment_size;
        if (last_received) received_bytes += payload.size() - (received.size() - 1) * segment_size;
    }

    const double progress = payload.empty() ? 1.0 : double(received_bytes) / double(payload.size());
    return {"receiver.message", "Incoming file", {
        {"file_name", "File name", "", path.filename().u8string()},
        {"file_size", "File size", "bytes", std::uint64_t(payload.size())},
        {"received_bytes", "Received bytes", "bytes", received_bytes},
        {"progress", "Progress", "", progress},
        {"completed", "Completed", "", complete()}
    }};
}

inline module_snapshot TxctrlQueue::inspect() const {
    return {"control.tx", "Outgoing control queue", {
        {"queued_packets", "Queued packets", "", std::uint64_t(packets.size())}
    }};
}

inline module_snapshot rxCtrlQueue::inspect() const {
    return {"control.rx", "Incoming control queue", {
        {"queued_packets", "Queued packets", "", std::uint64_t(packets.size())}
    }};
}

inline module_snapshot audio_device::inspect() const {
    return {"audio", "Audio device", {}};
}

class channel_rate_sampler {
public:
    using clock = std::chrono::steady_clock;

    void reset() { channels.clear(); }

    module_snapshot sample(module_snapshot snapshot, clock::time_point now = clock::now()) {
        for (auto& entry : channels) entry.second.seen = false;
        std::vector<inspect_field> rates;
        const std::string suffix = "accepted_payload_bytes";
        for (const auto& field : snapshot.fields) {
            if (field.id.size() < suffix.size() || field.id.compare(field.id.size() - suffix.size(), suffix.size(), suffix) != 0)
                continue;
            const auto* bytes = std::get_if<std::uint64_t>(&field.value);
            const std::string prefix = field.id.substr(0, field.id.size() - suffix.size());
            const auto* identity_value = inspection_detail::find_value(snapshot, prefix + "rate_identity");
            const auto* identity = identity_value ? std::get_if<std::string>(identity_value) : nullptr;
            if (!bytes || !identity) continue;

            auto& channel = channels[prefix];
            auto& history = channel.history;
            if (channel.identity != *identity || (!history.empty() && (*bytes < history.back().bytes || now < history.back().time)))
                history.clear();
            channel.identity = *identity;
            channel.seen = true;
            if (history.empty() || now > history.back().time) history.push_back({now, *bytes});
            const auto cutoff = now - std::chrono::seconds(3);
            while (history.size() > 2 && history[1].time <= cutoff) history.pop_front();
            while (history.size() > 128) history.pop_front();

            inspect_value rate = std::string("unavailable");
            if (history.size() > 1) {
                const double seconds = std::chrono::duration<double>(history.back().time - history.front().time).count();
                rate = 8.0 * double(history.back().bytes - history.front().bytes) / seconds;
            }
            rates.push_back({prefix + "bit_rate", "Decoded payload bit rate", "bits/s", std::move(rate)});
        }
        for (auto it = channels.begin(); it != channels.end();) {
            if (!it->second.seen) it = channels.erase(it);
            else ++it;
        }
        // Raw counters and instance identity are sampler inputs, not display fields.
        snapshot.fields.erase(std::remove_if(snapshot.fields.begin(), snapshot.fields.end(), [&](const inspect_field& field) {
            const auto dot = field.id.find_last_of('.');
            const auto start = dot == std::string::npos ? 0 : dot + 1;
            return field.id.compare(start, std::string::npos, suffix) == 0 ||
                   field.id.compare(start, std::string::npos, "rate_identity") == 0;
        }), snapshot.fields.end());
        snapshot.fields.reserve(snapshot.fields.size() + rates.size());
        for (auto& rate : rates) snapshot.fields.push_back(std::move(rate));
        return snapshot;
    }

private:
    struct sample_point { clock::time_point time; std::uint64_t bytes; };
    struct channel_history { std::string identity; std::deque<sample_point> history; bool seen = false; };
    std::unordered_map<std::string, channel_history> channels;
};

// One sampler per transfer, called on its owning worker. Reset before reusing it for a new transfer.
class transfer_sampler {
public:
    using clock = std::chrono::steady_clock;

    void reset() {
        history.clear();
        source.clear();
        file_name.clear();
        file_size = 0;
        channel_rates.reset();
    }

    module_snapshot sample(module_snapshot snapshot, clock::time_point now = clock::now()) {
        using inspection_detail::find_value;
        const std::string prefix = find_value(snapshot, "transfer.file_size") ? "transfer." : "";
        const auto* size_value = find_value(snapshot, prefix + "file_size");
        const auto* bytes_value = find_value(snapshot, prefix + "received_bytes");
        if (!bytes_value) bytes_value = find_value(snapshot, prefix + "acknowledged_bytes");
        const auto* name_value = find_value(snapshot, prefix + "file_name");
        const auto* size = size_value ? std::get_if<std::uint64_t>(size_value) : nullptr;
        const auto* bytes = bytes_value ? std::get_if<std::uint64_t>(bytes_value) : nullptr;
        const auto* name = name_value ? std::get_if<std::string>(name_value) : nullptr;
        if (!size || !bytes || !name) {
            history.clear();
            source.clear();
            file_name.clear();
            file_size = 0;
            return channel_rates.sample(std::move(snapshot), now);
        }

        const auto transferred = std::min(*bytes, *size);
        const auto remaining = *size - transferred;
        if (source != snapshot.id || file_name != *name || file_size != *size ||
            (!history.empty() && (transferred < history.back().bytes || now < history.back().time))) {
            reset();
            source = snapshot.id;
            file_name = *name;
            file_size = *size;
        }

        if (history.empty() || now > history.back().time) history.push_back({now, transferred});
        const auto cutoff = now - std::chrono::seconds(3);
        while (history.size() > 2 && history[1].time <= cutoff) history.pop_front();
        while (history.size() > 128) history.pop_front();

        std::optional<double> speed, eta;
        if (history.size() > 1) {
            const double seconds = std::chrono::duration<double>(history.back().time - history.front().time).count();
            speed = double(history.back().bytes - history.front().bytes) / seconds;
        }
        if (remaining == 0) eta = 0.0;
        else if (speed && *speed > 0.0) eta = double(remaining) / *speed;

        snapshot.fields.reserve(snapshot.fields.size() + 3);
        snapshot.fields.push_back({prefix + "remaining_bytes", "Remaining bytes", "bytes", remaining});
        snapshot.fields.push_back({prefix + "speed", "Transfer speed", "bytes/s",
            speed ? inspect_value(*speed) : inspect_value(std::string("unavailable"))});
        snapshot.fields.push_back({prefix + "eta", "Estimated time remaining", "s",
            eta ? inspect_value(*eta) : inspect_value(std::string("unavailable"))});
        return channel_rates.sample(std::move(snapshot), now);
    }

private:
    struct sample_point { clock::time_point time; std::uint64_t bytes; };
    std::deque<sample_point> history;
    std::string source, file_name;
    std::uint64_t file_size = 0;
    channel_rate_sampler channel_rates;
};

inline module_snapshot inspect(const sender& engine, transfer_sampler& sampler) {
    return sampler.sample(engine.inspect());
}

inline module_snapshot inspect(const receiver& engine, transfer_sampler& sampler) {
    return sampler.sample(engine.inspect());
}

// Poll on the engine worker; an empty result means no capture was due.
class inspection_monitor {
public:
    using clock = transfer_sampler::clock;

    explicit inspection_monitor(std::chrono::milliseconds interval = std::chrono::milliseconds(250)) {
        set_interval(interval);
    }

    void set_interval(std::chrono::milliseconds interval) {
        if (interval.count() <= 0) throw std::invalid_argument("Inspection interval must be positive");
        refresh_interval = interval;
    }

    void reset() {
        sampler.reset();
        last_capture.reset();
        source = nullptr;
    }

    template<typename Engine>
    std::optional<module_snapshot> poll(const Engine& engine, clock::time_point now = clock::now()) {
        if (source != &engine) {
            reset();
            source = &engine;
        }
        if (last_capture && now >= *last_capture && now - *last_capture < refresh_interval)
            return std::nullopt;
        auto snapshot = sampler.sample(engine.inspect(), now);
        last_capture = now;
        return snapshot;
    }

private:
    transfer_sampler sampler;
    std::chrono::milliseconds refresh_interval{250};
    std::optional<clock::time_point> last_capture;
    const void* source = nullptr;
};


namespace inspection_detail {

template<typename Packet, size_t Capacity>
class plot_queue {
public:
    static_assert(std::atomic<size_t>::is_always_lock_free, "Plot queues require lock-free indices");
    Packet* begin_write() {
        const auto next = written.load(std::memory_order_relaxed);
        if (next - read.load(std::memory_order_acquire) == Capacity) return nullptr;
        return &packets[next % Capacity];
    }
    void end_write() { written.store(written.load(std::memory_order_relaxed) + 1, std::memory_order_release); }
    const Packet* begin_read() {
        const auto next = read.load(std::memory_order_relaxed);
        if (next == written.load(std::memory_order_acquire)) return nullptr;
        return &packets[next % Capacity];
    }
    void end_read() { read.store(read.load(std::memory_order_relaxed) + 1, std::memory_order_release); }

private:
    std::array<Packet, Capacity> packets;
    std::atomic<size_t> written{0}, read{0};
};

}

struct waterfall_row {
    std::uint64_t sequence = 0, end_sample = 0, epoch = 0;
    double sample_rate = SAMPLE_RATE, bin_hz = 0.0;
    bool discontinuity = false;
    std::vector<float> power_dbfs;
};

struct constellation_frame {
    std::uintptr_t channel = 0;
    std::uint64_t epoch = 0;
    std::chrono::steady_clock::time_point analyzed_at;
    std::vector<cf32> points;
};

// One engine worker produces samples. Call analyze_pending() serially; stop capture and analysis before destruction.
class plot_inspector {
public:
    using clock = std::chrono::steady_clock;
    static constexpr size_t fft_size = 2048, max_grid = 4096, max_points = 512;

    plot_inspector() : fft_engine(fft_size) {
        for (size_t i = 0; i < fft_size; ++i) {
            window[i] = float(0.5 - 0.5 * std::cos(2.0 * pi * double(i) / double(fft_size - 1)));
            window_sum += window[i];
        }
    }

    plot_inspector(const plot_inspector&) = delete;
    plot_inspector& operator=(const plot_inspector&) = delete;

    void enable_waterfall(bool enabled) {
        auto old = audio_control.load(std::memory_order_relaxed);
        while (!audio_control.compare_exchange_weak(old, ((old + 2) & ~std::uint64_t(1)) | std::uint64_t(enabled))) {}
    }

    void select_constellation(std::uintptr_t channel) {
        selected_channel.store(channel, std::memory_order_release);
        constellation_epoch.fetch_add(1, std::memory_order_acq_rel);
    }

    void set_refresh(std::chrono::milliseconds waterfall, std::chrono::milliseconds constellation) {
        if (waterfall.count() <= 0 || constellation.count() <= 0)
            throw std::invalid_argument("Plot refresh intervals must be positive");
        waterfall_interval.store(waterfall.count(), std::memory_order_relaxed);
        constellation_interval.store(constellation.count(), std::memory_order_relaxed);
    }

    std::vector<waterfall_row> take_waterfall() {
        std::lock_guard<std::mutex> lock(output_mutex);
        std::vector<waterfall_row> result;
        result.swap(rows);
        const auto epoch = audio_control.load(std::memory_order_acquire);
        result.erase(std::remove_if(result.begin(), result.end(), [epoch](const waterfall_row& row) {
            return row.epoch != epoch;
        }), result.end());
        return result;
    }

    std::optional<constellation_frame> take_constellation() {
        std::lock_guard<std::mutex> lock(output_mutex);
        auto result = std::move(constellation_output);
        constellation_output.reset();
        if (result && (result->channel != selected_channel.load(std::memory_order_acquire) ||
                       result->epoch != constellation_epoch.load(std::memory_order_acquire))) result.reset();
        return result;
    }

    void analyze_pending(clock::time_point now = clock::now()) {
        const auto epoch = audio_control.load(std::memory_order_acquire);
        if (epoch != audio_epoch) {
            audio_epoch = epoch;
            expected_sample.reset();
            audio_filled = audio_write = 0;
            gap_pending = audio_dirty = false;
            emitted_waterfall = false;
        }
        for (size_t i = 0; i < 16; ++i) {
            const auto* packet = audio_queue.begin_read();
            if (!packet) break;
            if ((epoch & 1u) && packet->epoch == epoch) {
                consume_audio(*packet);
                audio_dirty = true;
            }
            audio_queue.end_read();
        }
        if (audio_dirty && audio_filled == fft_size && (!emitted_waterfall || now < last_waterfall ||
            now - last_waterfall >= std::chrono::milliseconds(waterfall_interval.load(std::memory_order_relaxed))))
            emit_waterfall(now);

        const auto current_epoch = constellation_epoch.load(std::memory_order_acquire);
        const auto selected = selected_channel.load(std::memory_order_acquire);
        if (current_epoch != last_constellation_epoch) {
            emitted_constellation = false;
            pending_constellation.reset();
            last_constellation_epoch = current_epoch;
        }
        for (size_t i = 0; i < 4; ++i) {
            const auto* packet = constellation_queue.begin_read();
            if (!packet) break;
            if (packet->epoch == current_epoch && packet->channel == selected) pending_constellation = *packet;
            constellation_queue.end_read();
        }
        if (pending_constellation && (pending_constellation->epoch != current_epoch || pending_constellation->channel != selected))
            pending_constellation.reset();
        if (pending_constellation && (!emitted_constellation || now < last_constellation || now - last_constellation >=
            std::chrono::milliseconds(constellation_interval.load(std::memory_order_relaxed)))) {
            emit_constellation(*pending_constellation, now);
            pending_constellation.reset();
        }
    }

    void capture_audio(const float* samples, size_t count) {
        const auto start = audio_position;
        audio_position += count;
        const auto epoch = audio_control.load(std::memory_order_acquire);
        if (!(epoch & 1u) || !samples) return;
        size_t offset = 0;
        while (offset < count) {
            auto* packet = audio_queue.begin_write();
            if (!packet) return;
            packet->count = std::min(count - offset, packet->samples.size());
            packet->start = start + offset;
            packet->epoch = epoch;
            std::memcpy(packet->samples.data(), samples + offset, packet->count * sizeof(float));
            offset += packet->count;
            audio_queue.end_write();
        }
    }

    void capture_constellation(const void* channel, cv32 samples, const slot& layout) {
        const auto id = reinterpret_cast<std::uintptr_t>(channel);
        const auto epoch = constellation_epoch.load(std::memory_order_acquire);
        if (!id || selected_channel.load(std::memory_order_acquire) != id) return;
        const auto nrb = layout.rb_vec.size();
        if (nrb < 6 || nrb > max_grid / (12 * NSYM_PER_RB) || !samples.re || !samples.im) return;
        auto* packet = constellation_queue.begin_write();
        if (!packet) return;
        packet->channel = id;
        packet->epoch = epoch;
        packet->nrb = nrb;
        packet->slot_type = layout.sltnmb;
        const auto count = nrb * 12 * NSYM_PER_RB;
        std::memcpy(packet->re.data(), samples.re, count * sizeof(float));
        std::memcpy(packet->im.data(), samples.im, count * sizeof(float));
        constellation_queue.end_write();
    }

private:
    struct audio_packet {
        std::array<float, 1024> samples;
        size_t count = 0;
        std::uint64_t start = 0, epoch = 0;
    };
    struct constellation_packet {
        std::array<float, max_grid> re{}, im{};
        size_t nrb = 0;
        u16 slot_type = 0;
        std::uintptr_t channel = 0;
        std::uint64_t epoch = 0;
    };
    inspection_detail::plot_queue<audio_packet, 16> audio_queue;
    inspection_detail::plot_queue<constellation_packet, 4> constellation_queue;
    std::atomic<std::uint64_t> audio_control{0}, constellation_epoch{0};
    std::atomic<std::uintptr_t> selected_channel{0};
    std::atomic<std::int64_t> waterfall_interval{100}, constellation_interval{250};
    std::uint64_t audio_position = 0;
    fft fft_engine;
    std::array<float, fft_size> window{}, audio_window{}, work_re{}, work_im{};
    double window_sum = 0.0;
    size_t audio_write = 0, audio_filled = 0;
    std::optional<std::uint64_t> expected_sample;
    bool gap_pending = false, audio_dirty = false;
    std::uint64_t audio_epoch = 0, row_sequence = 0;
    clock::time_point last_waterfall{}, last_constellation{};
    bool emitted_waterfall = false, emitted_constellation = false;
    std::mutex output_mutex;
    std::vector<waterfall_row> rows;
    std::optional<constellation_frame> constellation_output;
    std::optional<constellation_packet> pending_constellation;
    std::uint64_t last_constellation_epoch = 0;

    void consume_audio(const audio_packet& packet) {
        if (expected_sample && *expected_sample != packet.start) {
            audio_filled = 0;
            audio_write = 0;
            gap_pending = true;
        }
        for (size_t i = 0; i < packet.count; ++i) {
            audio_window[audio_write] = packet.samples[i];
            audio_write = (audio_write + 1) % fft_size;
        }
        audio_filled = std::min(fft_size, audio_filled + packet.count);
        expected_sample = packet.start + packet.count;
    }

    void emit_waterfall(clock::time_point now) {
        for (size_t i = 0; i < fft_size; ++i) {
            work_re[i] = audio_window[(audio_write + i) % fft_size] * window[i];
            work_im[i] = 0.0f;
        }
        fft_engine.forward({work_re.data(), work_im.data()});
        waterfall_row row;
        row.sequence = row_sequence++;
        row.end_sample = *expected_sample;
        row.epoch = audio_epoch;
        row.bin_hz = double(SAMPLE_RATE) / fft_size;
        row.discontinuity = gap_pending;
        row.power_dbfs.resize(fft_size / 2 + 1);
        for (size_t i = 0; i < row.power_dbfs.size(); ++i) {
            const double scale = (i == 0 || i == fft_size / 2) ? 1.0 : 2.0;
            const double re = work_re[i], im = work_im[i];
            const double power = (re * re + im * im) * scale * scale / (window_sum * window_sum);
            row.power_dbfs[i] = float(10.0 * std::log10(std::max(1e-16, power)));
        }
        {
            std::lock_guard<std::mutex> lock(output_mutex);
            if (rows.size() == 128) rows.erase(rows.begin());
            rows.push_back(std::move(row));
        }
        gap_pending = false;
        audio_dirty = false;
        last_waterfall = now;
        emitted_waterfall = true;
    }

    void emit_constellation(const constellation_packet& packet, clock::time_point now) {
        slot layout(packet.slot_type, u16(packet.nrb), QPSK);
        constellation_frame frame;
        frame.channel = packet.channel;
        frame.epoch = packet.epoch;
        frame.analyzed_at = now;
        frame.points.reserve(max_points);
        const size_t stride = std::max(size_t(1), (size_t(layout.capacity()) / 2 + max_points - 1) / max_points);
        size_t data_index = 0;
        const size_t nsc = packet.nrb * 12;
        for (u16 symbol = 0; symbol < NSYM_PER_RB; ++symbol) {
            for (size_t sc = 0; sc < nsc; ++sc) {
                const auto& rb = layout.rb_vec[sc / 12];
                if (rb.get_label(u16(sc % 12), symbol) != RE_type::D) continue;
                if ((data_index++ % stride) != 0) continue;
                const auto index = symbol * nsc + sc;
                if (std::isfinite(packet.re[index]) && std::isfinite(packet.im[index]))
                    frame.points.emplace_back(packet.re[index], packet.im[index]);
            }
        }
        {
            std::lock_guard<std::mutex> lock(output_mutex);
            constellation_output = std::move(frame);
        }
        last_constellation = now;
        emitted_constellation = true;
    }


};

inline thread_local plot_inspector* active_plot_inspector = nullptr;

// Bind and unbind on the engine worker. The inspector must outlive this scope.
class plot_capture_scope {
public:
    explicit plot_capture_scope(plot_inspector& inspector) : previous(active_plot_inspector) { active_plot_inspector = &inspector; }
    ~plot_capture_scope() { active_plot_inspector = previous; }
    plot_capture_scope(const plot_capture_scope&) = delete;
    plot_capture_scope& operator=(const plot_capture_scope&) = delete;
private:
    plot_inspector* previous;
};

inline void audio_device::inspect_audio(const float* samples, size_t count) const {
    if (active_plot_inspector) active_plot_inspector->capture_audio(samples, count);
}

inline void slot_demapper::inspect_constellation(cv32 samples, const slot& layout) const {
    if (active_plot_inspector) active_plot_inspector->capture_constellation(this, samples, layout);
}

} // namespace soundlink

#endif
