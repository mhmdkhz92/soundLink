# SoundLink GUI shell

This directory retains a minimal Qt Quick application for the next interface.
The previous standalone transmitter/receiver adapters, controls, diagnostics,
and plots have been removed. The modem implementation remains in `../src`.

- `main.cpp` starts the application and loads `Main.qml`.
- `Main.qml` is the root window for future UI components.
- `CMakeLists.txt` builds the QML module and packages the application.
- The plist templates and icon catalog retain macOS/iOS packaging scaffolding.

The shell does not start audio, instantiate modem chains, or expose a backend
to QML. Add the future controller to the executable and register it in
`main.cpp` when the protocol is ready.

## Build on macOS

```sh
/Users/mohammad/Qt/Tools/CMake/CMake.app/Contents/bin/cmake -S gui -B build/gui -DCMAKE_PREFIX_PATH=/Users/mohammad/Qt/6.12.0/macos
/Users/mohammad/Qt/Tools/CMake/CMake.app/Contents/bin/cmake --build build/gui
```

Use your installed CMake and Qt paths on other machines. Alternatively, use the
Qt kit's `qt-cmake` wrapper when CMake is on your PATH. Existing generated build
directories may be reconfigured; a fresh build directory is simplest.

## iOS scaffold

Generate an Xcode project using the installed iOS Qt kit, for example:

```sh
/Users/mohammad/Qt/6.12.0/ios_device/bin/qt-cmake -S gui -B build/ios-device -G Xcode -DCMAKE_OSX_SYSROOT=iphoneos -DCMAKE_OSX_ARCHITECTURES=arm64 -DQT_HOST_PATH=/Users/mohammad/Qt/6.12.0/macos
```

Select signing and your device in Xcode. Microphone permission declarations are
retained for future audio integration; the shell itself does not request access.
