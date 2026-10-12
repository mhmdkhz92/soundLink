#include "receiver.h"
#if defined(Q_OS_IOS)
#import <AVFoundation/AVFoundation.h>
#endif
#include "receive_rate.h"
#include "snr_estimate.h"
#define MINIAUDIO_IMPLEMENTATION
#include "../src/miniaudio.h"
#include "../src/math.h"
#include "../src/dsp.h"
#include "../src/generic.h"
#include "../src/perm.h"
#include "../src/viterbi.h"
#include <QTimer>
#include <QCoreApplication>
#include <QPermissions>
#include <QVariantMap>
#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

namespace {
using namespace soundlink;
using Clock = std::chrono::steady_clock;
static_assert(std::atomic<ma_uint32>::is_always_lock_free,
              "Capture callback requires a lock-free dropout counter");
struct Snapshot {
    QString status = "Stopped";
    uint64_t bits = 0, errors = 0, dropped = 0, clipped = 0;
    double recent = -1, level = -120, dataRate = -1;
    QVariantList spectrum, constellation, rbSnr;
};
struct Chain {
    scheduler rx;
    link_cfg cfg;
    pipebuf<float> audio{&rx, "microphone", 262144};
    pipebuf_c<float> baseband{&rx, "baseband", 65536};
    pipebuf_c<float> grid{&rx, "grid", 65536};
    pipebuf<u8> soft{&rx, "soft", 65536};
    pipebuf<u8> decoded{&rx, "viterbi", 65536};
    pipebuf<u8> deinterleaved{&rx, "deinterleaved", 65536};
    pipebuf<u8> payload{&rx, "payload", 65536};
    pipewriter<float> input{audio};
    ddc down;
    ofdm_demodulator ofdm;
    slot_demapper demap;
    viterbi decoder;
    deinterleaver<127, 24> interleave;
    bch_decoder<bch127_106> bch;
    ber_track ber;
    std::vector<cf32> points;
    Chain(BW bw, code_rate rate, int fc)
        : cfg(bw), down(&rx, audio, baseband, cfg),
          ofdm(&rx, baseband, grid, cfg), demap(&rx, grid, soft, cfg),
          decoder(&rx, soft, decoded, rate),
          interleave(&rx, decoded, deinterleaved),
          bch(&rx, deinterleaved, payload), ber(&rx, payload) {
        down.set_fc(fc);
        demap.observer_context = this;
        points.reserve(1024);
        demap.observe_symbol = nullptr;
    }
    static void observe(void* context, cf32 sample) {
        auto* self = static_cast<Chain*>(context);
        if (self->points.size() < 1024) self->points.push_back(sample);
    }
};
}

struct Receiver::Impl {
    ma_pcm_rb ring{};
    ma_device device{};
#if defined(Q_OS_IOS)
    ma_context context{};
#endif
    bool active = false;
    bool permissionPending = false;
    std::atomic<bool> stopping{false};
    std::atomic<int> display{0};
    std::atomic<ma_uint32> dropped{0};
    std::thread worker;
    std::mutex mutex;
    Snapshot latest, shown;
    QTimer timer;

    static void capture(ma_device* device, void*, const void* input, ma_uint32 frames) {
        auto& self = *static_cast<Impl*>(device->pUserData);
        if (!input) {
            self.dropped.fetch_add(frames);
            return;
        }
        const auto* samples = static_cast<const float*>(input);
        ma_uint32 copied = 0;
        while (copied < frames) {
            ma_uint32 count = frames - copied;
            void* memory = nullptr;
            if (ma_pcm_rb_acquire_write(&self.ring, &count, &memory) != MA_SUCCESS || !count)
                break;
            std::memcpy(memory, samples + copied, count * sizeof(float));
            ma_pcm_rb_commit_write(&self.ring, count);
            copied += count;
        }
        if (copied < frames) self.dropped.fetch_add(frames - copied);
    }

    void process(BW bw, code_rate rate, int fc) {
        auto chain = std::make_unique<Chain>(bw, rate, fc);
        Snapshot state;
        uint64_t previousBits = 0, previousErrors = 0;
        uint64_t savedBits = 0, savedErrors = 0;
        ma_uint32 lastDropped = 0;
        struct Count { Clock::time_point time; uint64_t bits, errors; };
        std::deque<Count> history;
        ReceiveRate receivedRate;
        auto next = Clock::now();
        float re[1024]{}, im[1024]{}, samplesForSpectrum[1024]{};
        float window[1024];
        for (size_t j = 0; j < 1024; ++j)
            window[j] = 0.5f - 0.5f * std::cos(2 * pi * j / 1023);
        size_t spectrumWrite = 0;
        size_t sampleCount = 0;
        double energy = 0;
        uint64_t levelCount = 0;
        fft transform(1024);
        while (!stopping.load()) {
            const int visiblePlot = display.load();
            if (visiblePlot != 1) chain->points.clear();
            if (visiblePlot != 0) sampleCount = 0;
            const ma_uint32 lost = dropped.load();
            if (lost != lastDropped) {
                // A sample gap invalidates all downstream decoder state.
                savedBits += chain->ber.compared_bits;
                savedErrors += chain->ber.error_bits;
                chain = std::make_unique<Chain>(bw, rate, fc);
                state.dropped += static_cast<ma_uint32>(lost - lastDropped);
                lastDropped = lost;
                // Discard queued pre-gap samples, rather than splice them
                // to post-gap samples in the freshly reset decoder.
                ma_uint32 discard = ma_pcm_rb_available_read(&ring);
                while (discard) {
                    ma_uint32 section = discard;
                    void* old = nullptr;
                    if (ma_pcm_rb_acquire_read(&ring, &section, &old) != MA_SUCCESS || !section) break;
                    ma_pcm_rb_commit_read(&ring, section);
                    state.dropped += section;
                    discard -= section;
                }
                history.clear();
                receivedRate.reset();
                sampleCount = 0;
            }
            chain->demap.observe_symbol = visiblePlot == 1 ? Chain::observe : nullptr;
            ma_uint32 count = static_cast<ma_uint32>(
                std::min<size_t>(4096, chain->input.writable()));
            void* memory = nullptr;
            if (count && ma_pcm_rb_acquire_read(&ring, &count, &memory) == MA_SUCCESS && count) {
                auto* samples = static_cast<float*>(memory);
                for (ma_uint32 i = 0; i < count; ++i) {
                    const float value = samples[i];
                    energy += double(value) * value;
                    ++levelCount;
                    if (std::fabs(value) >= 0.999f) ++state.clipped;
                    if (visiblePlot == 0) {
                        samplesForSpectrum[spectrumWrite] = value;
                        spectrumWrite = (spectrumWrite + 1) % 1024;
                        sampleCount = std::min<size_t>(sampleCount + 1, 1024);
                    }
                }
                std::memcpy(chain->input.wr(), memory, count * sizeof(float));
                chain->input.written(count);
                ma_pcm_rb_commit_read(&ring, count);
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            chain->rx.run();
            const auto now = Clock::now();
            receivedRate.observe(now, chain->ber.locked, chain->ber.compared_bits);
            if (now >= next) {
                if (visiblePlot == 0 && sampleCount == 1024) {
                    for (size_t j = 0; j < 1024; ++j) {
                        re[j] = samplesForSpectrum[(spectrumWrite + j) % 1024] * window[j];
                        im[j] = 0;
                    }
                    transform.forward({re, im});
                    state.spectrum.clear();
                    state.spectrum.reserve(513);
                    for (size_t j = 0; j <= 512; ++j) {
                        const double magnitude = std::hypot(re[j], im[j]) / 512.0;
                        state.spectrum.append(QVariantMap{
                            {"frequency", double(j) * 48000 / 1024},
                            {"db", 20 * std::log10(std::max(magnitude, 1e-6))}});
                    }
                }
                state.bits = savedBits + chain->ber.compared_bits;
                state.errors = savedErrors + chain->ber.error_bits;
                history.push_back({now, state.bits - previousBits, state.errors - previousErrors});
                previousBits = state.bits;
                previousErrors = state.errors;
                while (!history.empty() && now - history.front().time > std::chrono::seconds(1))
                    history.pop_front();
                uint64_t bits = 0, errors = 0;
                for (const auto& entry : history) { bits += entry.bits; errors += entry.errors; }
                state.recent = bits ? double(errors) / bits : -1;
                state.dataRate = receivedRate.value();
                state.status = chain->ber.locked ? "Measuring BER" :
                    chain->ofdm.synchronized() ? "Acquiring PRBS" : "Searching";
                state.level = levelCount ? 10 * std::log10(std::max(energy / levelCount, 1e-12)) : -120;
                energy = 0; levelCount = 0;
                if (visiblePlot == 2) {
                    state.rbSnr.clear();
                    const auto& stats = chain->demap.equalizer_stats();
                    state.rbSnr.reserve(int(stats.size()));
                    for (size_t rb = 0; rb < stats.size(); ++rb) {
                        const auto estimate = estimateQpskSnr(stats[rb].m2, stats[rb].m4, stats[rb].noise);
                        const bool valid = chain->ofdm.synchronized() && estimate.valid;
                        state.rbSnr.append(QVariantMap{{"rb", int(rb + 1)},
                            {"valid", valid}, {"db", valid ? QVariant(estimate.db) : QVariant()},
                            {"capped", valid && estimate.capped}});
                    }
                }
                if (visiblePlot == 1 && !chain->points.empty()) {
                    state.constellation.clear();
                    state.constellation.reserve(int(chain->points.size()));
                    for (const auto& point : chain->points)
                        state.constellation.append(QVariantMap{{"x", point.real()}, {"y", point.imag()}});
                    chain->points.clear();
                }
                // Never wait for the GUI to accept a display snapshot.
                std::unique_lock<std::mutex> lock(mutex, std::try_to_lock);
                if (lock.owns_lock()) latest = state;
                next = now + std::chrono::milliseconds(100);
            }
        }
    }
};

Receiver::Receiver(QObject* parent) : QObject(parent), impl(std::make_unique<Impl>()) {
    connect(&impl->timer, &QTimer::timeout, this, &Receiver::refresh);
    impl->timer.start(100);
}
Receiver::~Receiver() { stop(); }
bool Receiver::running() const { return impl->active; }
QString Receiver::status() const { return impl->shown.status; }
double Receiver::totalBer() const { return impl->shown.bits ? double(impl->shown.errors) / impl->shown.bits : -1; }
double Receiver::recentBer() const { return impl->shown.recent; }
double Receiver::dataRate() const { return impl->active ? impl->shown.dataRate : -1; }
qulonglong Receiver::comparedBits() const { return impl->shown.bits; }
qulonglong Receiver::errorBits() const { return impl->shown.errors; }
qulonglong Receiver::droppedFrames() const { return impl->shown.dropped; }
qulonglong Receiver::clippedSamples() const { return impl->shown.clipped; }
double Receiver::inputLevel() const { return impl->shown.level; }
QVariantList Receiver::spectrum() const { return impl->shown.spectrum; }
QVariantList Receiver::constellation() const { return impl->shown.constellation; }
QVariantList Receiver::rbSnr() const { return impl->shown.rbSnr; }
void Receiver::setDisplay(int plot) {
    impl->display.store(plot >= 0 && plot <= 2 ? plot : -1);
}
void Receiver::refresh() {
    if (!impl->active) return;
    { std::lock_guard<std::mutex> lock(impl->mutex); impl->shown = impl->latest; }
    emit updated();
}
bool Receiver::start(double bandwidth, QString rate, int fc) {
    if (impl->active) return false;
    BW bw;
    if (bandwidth == 1.4) bw = BW::BW1_4;
    else if (bandwidth == 3) bw = BW::BW3;
    else if (bandwidth == 5) bw = BW::BW5;
    else if (bandwidth == 10) bw = BW::BW10;
    else { impl->shown.status = "Invalid bandwidth"; emit updated(); return false; }
    code_rate cr;
    if (rate == "1/2") cr = FEC12;
    else if (rate == "2/3") cr = FEC23;
    else if (rate == "3/4") cr = FEC34;
    else if (rate == "5/6") cr = FEC56;
    else if (rate == "7/8") cr = FEC78;
    else { impl->shown.status = "Invalid code rate"; emit updated(); return false; }
    if (fc < 2 || fc > 18) { impl->shown.status = "Invalid center frequency"; emit updated(); return false; }
#if QT_CONFIG(permissions)
    const QMicrophonePermission permission;
    const auto permissionStatus = qApp->checkPermission(permission);
    if (permissionStatus == Qt::PermissionStatus::Denied) {
        impl->shown.status = "Microphone permission denied"; emit updated(); return false;
    }
    if (permissionStatus == Qt::PermissionStatus::Undetermined) {
        if (impl->permissionPending) return false;
        impl->permissionPending = true;
        impl->shown.status = "Waiting for microphone permission"; emit updated();
        qApp->requestPermission(permission, this, [this, bandwidth, rate, fc](const QPermission& result) {
            if (!impl->permissionPending) return;
            impl->permissionPending = false;
            if (result.status() == Qt::PermissionStatus::Granted) start(bandwidth, rate, fc);
            else { impl->shown.status = "Microphone permission denied"; emit updated(); }
        });
        return false;
    }
#endif
    if (ma_pcm_rb_init(ma_format_f32, 1, 48000, nullptr, nullptr, &impl->ring) != MA_SUCCESS) {
        impl->shown.status = "Could not allocate capture ring"; emit updated(); return false;
    }
    auto config = ma_device_config_init(ma_device_type_capture);
    config.capture.format = ma_format_f32;
    config.capture.channels = 1;
    config.sampleRate = 48000;
    config.dataCallback = Impl::capture;
    config.pUserData = impl.get();
    ma_context* context = nullptr;
#if defined(Q_OS_IOS)
    auto contextConfig = ma_context_config_init();
    contextConfig.coreaudio.sessionCategory = ma_ios_session_category_record;
    if (ma_context_init(nullptr, 0, &contextConfig, &impl->context) != MA_SUCCESS) {
        ma_pcm_rb_uninit(&impl->ring);
        impl->shown.status = "Could not start audio session"; emit updated(); return false;
    }
    context = &impl->context;
    NSError* audioError = nil;
    if (![[AVAudioSession sharedInstance] setMode:AVAudioSessionModeMeasurement error:&audioError]) {
        ma_context_uninit(context);
        ma_pcm_rb_uninit(&impl->ring);
        impl->shown.status = "Could not enable measurement mode"; emit updated(); return false;
    }
#endif
    if (ma_device_init(context, &config, &impl->device) != MA_SUCCESS) {
#if defined(Q_OS_IOS)
        ma_context_uninit(&impl->context);
#endif
        ma_pcm_rb_uninit(&impl->ring);
        impl->shown.status = "Could not open microphone"; emit updated(); return false;
    }
    impl->stopping.store(false); impl->dropped.store(0);
    impl->latest = Snapshot{}; impl->latest.status = "Searching";
    impl->shown = impl->latest;
    impl->worker = std::thread([this, bw, cr, fc] { impl->process(bw, cr, fc); });
    if (ma_device_start(&impl->device) != MA_SUCCESS) {
        ma_device_uninit(&impl->device);
#if defined(Q_OS_IOS)
        ma_context_uninit(&impl->context);
#endif
        impl->stopping.store(true); impl->worker.join();
        ma_pcm_rb_uninit(&impl->ring);
        impl->shown.status = "Could not start microphone"; emit updated(); return false;
    }
    impl->active = true; emit updated(); return true;
}
void Receiver::stop() {
    const bool wasPending = impl->permissionPending;
    impl->permissionPending = false;
    if (!impl->active) {
        if (wasPending) {
            impl->shown.status = "Stopped";
            emit updated();
        }
        return;
    }
    ma_device_uninit(&impl->device);
#if defined(Q_OS_IOS)
        ma_context_uninit(&impl->context);
#endif
    impl->stopping.store(true);
    impl->worker.join();
    ma_pcm_rb_uninit(&impl->ring);
    { std::lock_guard<std::mutex> lock(impl->mutex); impl->shown = impl->latest; }
    impl->active = false; impl->shown.status = "Stopped"; emit updated();
}

// Shared DSP headers define non-inline functions. Compile both adapters once
// in this translation unit to use them without editing protected headers.
#include "transmitter.inc"
