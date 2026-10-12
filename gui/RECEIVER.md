# Receiver backend

`receiver.cpp` runs the RX chain inside the Qt application. No separate RX
command-line executable is needed. `main.cpp` exposes it to QML as `receiver`.
`Main.qml` supplies controls, spectrum and constellation plots, BER cards,
and audio health indicators. Settings are disabled during reception; starting
a new session resets statistics.

Example QML calls:

```qml
receiver.start(5, "1/2", 5) // bandwidth kHz, code rate, center frequency kHz
receiver.stop()
```

`start()` requests microphone permission asynchronously when necessary. In that
case it returns false initially and resumes startup after permission is granted.
Use `running` and `status` to observe the actual result. The default microphone
is used; selecting a device and handling route changes are future work.
Starting a new session resets all display statistics.

Properties are updated at 10 Hz on the GUI thread:

- `running`, `status`: session and acquisition state.
- `totalBer`, `recentBer`: total BER and approximately one second of recent BER.
  A value of -1 means no valid compared bits; render it as an em dash.
- `comparedBits`, `errorBits`: cumulative PRBS tracker counters.
- `dataRate`: valid compared payload bits per wall-clock second over approximately
  one second of continuous PRBS lock (not encoded/channel bit rate). A value of
  -1 means unlocked, stopped, or less than 250 ms of measurement time. A loss of
  lock or capture discontinuity resets the rate window.
- `spectrum`: maps containing `frequency` (Hz) and `db` (amplitude relative to
  full scale). A 1024-point Hann-windowed FFT covers 0 to 24 kHz. This is a
  display spectrum, not a calibrated noise/SNR estimator.
- `constellation`: maps containing `x` and `y`, capped at 1024 equalized data
  symbols per update. Pilot and synchronization symbols are excluded.
  The last nonempty constellation remains visible until new points arrive;
  starting a new session clears it.
- `inputLevel`: RMS level in dBFS over the update interval.
- `clippedSamples`: samples with absolute amplitude at least 0.999.
- `droppedFrames`: capture-ring losses plus queued frames intentionally discarded
  on recovery. This does not measure all possible hardware/driver dropouts.

The microphone callback only copies to a mono float 48 kHz ring. Only the DSP
worker accesses scheduler pipes. On a capture-ring discontinuity the worker
rebuilds the decoder chain and discards the queued audio to prevent decoding
across a known gap. Previously compared BER counts are retained. GUI snapshots
use a separate mutex; the DSP worker skips publication rather than waiting.

The GUI and command-line transmitter share the top-level `src` folder.
The shared DSP headers expose OFDM synchronization status and an optional
equalized-symbol observer without any Qt dependency. There is no `gui/src` copy.
`generic.h` retains its POSIX file utilities.
## Mac and iPhone

Both targets use the same Main.qml and receiver backend, and the same protected
../src headers. Narrow windows stack the plots and use two columns for metrics;
wide windows retain larger plot space. Spectrum and constellation share a
stack with a tab selector, so only one plot is active. ApplicationWindow supplies iOS safe-area
padding. Reception stops when the iOS app becomes inactive; tap Start to resume.
The iOS audio session uses Record with Measurement mode to minimize voice processing.

Configure an iPhone-only Xcode project (no simulator needed):

```sh
/Users/mohammad/Qt/6.12.0/ios_device/bin/qt-cmake -S gui -B gui/build/ios-device -G Xcode -DCMAKE_OSX_SYSROOT=iphoneos -DCMAKE_OSX_ARCHITECTURES=arm64 -DQT_HOST_PATH=/Users/mohammad/Qt/6.12.0/macos
```

Open gui/build/ios-device/gui.xcodeproj in Xcode. Select target appgui,
Signing & Capabilities, enable automatic signing and select your Personal Team.
Use a unique bundle identifier if required. Connect your iPhone, select it as
the run destination, and press Run. Enable Developer Mode on the phone if prompted.
The current Qt kit sets the minimum deployment version to iOS 18.

For a compile-only check without signing:

```sh
/Users/mohammad/Qt/Tools/CMake/CMake.app/Contents/bin/cmake --build gui/build/ios-device --config Release --parallel 4 -- CODE_SIGNING_ALLOWED=NO
```

An unsigned build cannot be installed on a phone. Actual microphone reception,
interruption recovery and hardware sample-rate conversion still need a device test.



## Receiver and transmitter tabs

Switching the main tab stops the current audio session. The transmitter uses
the same PRBS, BCH, interleaver, convolutional encoder, OFDM mapper/modulator,
DUC and speaker sink as ../soundlink_tx.cpp, with the existing shared headers.
There is no full-duplex mode. Speaker setup and the 250 ms ring prefill run on
a worker thread; missing output frames are zero-filled and counted, with no
printing or allocations in the audio callback. Output volume uses device controls.

The spectrum keeps the latest 1024 input samples and performs one FFT and
display conversion per 100 ms at most. Its Hann window is precomputed. When
constellation is selected, the spectrum is suspended; when spectrum is selected,
the equalized-symbol observer is disabled. Constellation samples use a bounded
raw complex vector and are converted to Qt maps only at the 10 Hz update. The
last displayed points remain until fresh symbols arrive. Display selection
does not interrupt RX decoding or BER statistics.

transmitter.inc is included by receiver.cpp intentionally: some protected DSP
headers contain non-inline function definitions, so both adapters share one
translation unit. No shared header needs modification or a duplicate copy.

## SNR per RB tile

The receiver has a third plot tab, SNR / RB. It reads the public
slot_demapper::equalizer_stats() accessor on the DSP worker and publishes
SNR maps at most every 100 ms while this tile is selected. No extra work is
performed by the microphone callback. Spectrum and constellation remain
suspended while the SNR tile is active.

For the current QPSK receiver, signal power is m2 minus stored noise, and
SNR is 10 log10(signal/noise). Uninitialized, nonfinite, invalid-moment or
inconsistent stored-noise estimates are shown as unavailable. The consistency
check uses the QPSK moment formula because the shared estimator can retain
old noise after updating its moments. This is an approximate post-equalization
estimate, including channel-estimation errors and distortion, not calibrated
physical-channel SNR. A future modulation change must update this check.

The bars use RB numbering 1 through nRB (internal index plus one), on a fixed
-10 to 60 dB scale. Estimates at or above 60 dB are capped and the tapped value
is prefixed with >=. Tap a bar for its value; gray indicates unavailable.
Starting a new session clears the plot; stopped reception retains the last
snapshot. This feature adds no modifications to src.
