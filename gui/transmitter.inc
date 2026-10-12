// Included by receiver.cpp: a single translation unit owns shared DSP definitions.
#include "transmitter.h"
#include "../src/device.h"
#include <QMetaObject>

namespace {
struct TxChain {
    scheduler tx;
    link_cfg cfg;
    pipebuf<u8> data{&tx, "input", 8096};
    pipebuf<u8> encoded{&tx, "bch", 8096};
    pipebuf<u8> interleaved{&tx, "interleaved", 8096};
    pipebuf<u8> convolutional{&tx, "convolutional", 8096};
    pipebuf_c<float> slotGrid{&tx, "slotGrid", 3*8096};
    pipebuf_c<float> time{&tx, "ifft", 3*8096};
    pipebuf<float> audio{&tx, "48kHz", 3*8096};
    prbs_generator prbs{&tx, data};
    bch_encoder<bch127_106> bch{&tx, data, encoded};
    interleaver<127, 24> interleave{&tx, encoded, interleaved};
    conv_encoder encoder;
    slot_mapper mapper;
    ofdm_modulator modulator;
    duc up;
    speaker_sink sink;
    TxChain(BW bw, code_rate cr, int fc, ma_pcm_rb& ring)
        : cfg(bw), encoder(&tx, interleaved, convolutional, cr),
          mapper(&tx, convolutional, slotGrid, cfg), modulator(&tx, slotGrid, time, cfg),
          up(&tx, time, audio, cfg), sink(&tx, audio, ring) { up.set_fc(fc); }
};
}

struct Transmitter::Impl {
    bool active = false;
    QString status = "Stopped";
    unsigned generation = 0;
    std::atomic<bool> stopping{false};
    std::atomic<ma_uint32> underruns{0};
    std::thread worker;
    ma_pcm_rb ring{};
    QTimer timer;

    static void playback(ma_device* device, void* output, const void*, ma_uint32 frames) {
        auto& self = *static_cast<Impl*>(device->pUserData);
        auto* samples = static_cast<float*>(output);
        ma_uint32 copied = 0;
        while (copied < frames) {
            ma_uint32 count = frames - copied;
            void* memory = nullptr;
            if (ma_pcm_rb_acquire_read(&self.ring, &count, &memory) != MA_SUCCESS || !count) break;
            std::memcpy(samples + copied, memory, count * sizeof(float));
            ma_pcm_rb_commit_read(&self.ring, count);
            copied += count;
        }
        if (copied < frames) {
            std::memset(samples + copied, 0, (frames - copied) * sizeof(float));
            self.underruns.fetch_add(frames - copied);
        }
    }
};
Transmitter::Transmitter(QObject* parent) : QObject(parent), impl(std::make_unique<Impl>()) {
    connect(&impl->timer, &QTimer::timeout, this, [this] { if (impl->active) emit updated(); });
    impl->timer.start(100);
}
Transmitter::~Transmitter() { stop(); }
bool Transmitter::running() const { return impl->active; }
QString Transmitter::status() const { return impl->status; }
qulonglong Transmitter::underrunFrames() const { return impl->underruns.load(); }
bool Transmitter::start(double bandwidth, QString rate, int fc) {
    if (impl->active) return false;
    BW bw;
    code_rate cr;
    if (bandwidth == 1.4) bw = BW::BW1_4;
    else if (bandwidth == 3) bw = BW::BW3;
    else if (bandwidth == 5) bw = BW::BW5;
    else if (bandwidth == 10) bw = BW::BW10;
    else { impl->status = "Invalid bandwidth"; emit updated(); return false; }
    if (rate == "1/2") cr = FEC12;
    else if (rate == "2/3") cr = FEC23;
    else if (rate == "3/4") cr = FEC34;
    else if (rate == "5/6") cr = FEC56;
    else if (rate == "7/8") cr = FEC78;
    else { impl->status = "Invalid code rate"; emit updated(); return false; }
    if (fc < 2 || fc > 18) { impl->status = "Invalid center frequency"; emit updated(); return false; }
    if (impl->worker.joinable()) impl->worker.join();
    impl->active = true;
    impl->status = "Starting speaker";
    impl->stopping.store(false);
    impl->underruns.store(0);
    const unsigned generation = ++impl->generation;
    emit updated();
    impl->worker = std::thread([this, bw, cr, fc, generation] {
        auto publish = [this, generation](QString status, bool running) {
            QMetaObject::invokeMethod(this, [this, generation, status, running] {
                if (generation != impl->generation) return;
                impl->status = status; impl->active = running; emit updated();
            }, Qt::QueuedConnection);
        };
        if (ma_pcm_rb_init(ma_format_f32, 1, CAPACITY, nullptr, nullptr, &impl->ring) != MA_SUCCESS) {
            publish("Could not allocate playback ring", false); return;
        }
        ma_context* context = nullptr;
#if defined(Q_OS_IOS)
        ma_context iosContext{};
        auto contextConfig = ma_context_config_init();
        contextConfig.coreaudio.sessionCategory = ma_ios_session_category_playback;
        if (ma_context_init(nullptr, 0, &contextConfig, &iosContext) != MA_SUCCESS) {
            ma_pcm_rb_uninit(&impl->ring);
            publish("Could not start audio session", false); return;
        }
        context = &iosContext;
        [[AVAudioSession sharedInstance] setMode:AVAudioSessionModeDefault error:nil];
#endif
        ma_device device{};
        auto config = ma_device_config_init(ma_device_type_playback);
        config.playback.format = ma_format_f32;
        config.playback.channels = 1;
        config.sampleRate = SAMPLE_RATE;
        config.dataCallback = Impl::playback;
        config.pUserData = impl.get();
        const bool opened = ma_device_init(context, &config, &device) == MA_SUCCESS;
        QString failure;
        if (!opened) failure = "Could not open speaker";
        if (opened) {
            TxChain chain(bw, cr, fc, impl->ring);
            while (!impl->stopping.load() && ma_pcm_rb_available_read(&impl->ring) < TARGET_FILL) {
                const auto before = chain.tx.hash();
                chain.tx.step();
                if (chain.tx.hash() == before) { failure = "TX stalled during prefill"; break; }
            }
            if (failure.isEmpty() && !impl->stopping.load()) {
                if (ma_device_start(&device) != MA_SUCCESS) failure = "Could not start speaker";
                else {
                    publish("Transmitting", true);
                    while (!impl->stopping.load()) {
                        if (ma_pcm_rb_available_read(&impl->ring) >= SLEEP_THRESHOLD) {
                            std::this_thread::sleep_for(std::chrono::milliseconds(20)); continue;
                        }
                        const auto before = chain.tx.hash();
                        chain.tx.step();
                        if (chain.tx.hash() == before)
                            std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    }
                }
            }
            ma_device_uninit(&device);
        }
#if defined(Q_OS_IOS)
        ma_context_uninit(&iosContext);
#endif
        ma_pcm_rb_uninit(&impl->ring);
        if (!failure.isEmpty()) publish(failure, false);
    });
    return true;
}
void Transmitter::stop() {
    ++impl->generation;
    impl->stopping.store(true);
    if (impl->worker.joinable()) impl->worker.join();
    impl->active = false; impl->status = "Stopped"; emit updated();
}
