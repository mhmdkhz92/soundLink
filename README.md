# SoundLink

SoundLink is an acoustic modem that sends a file between two computers through their speakers and microphones. It turns file data into audio and sends acknowledgments back over the same acoustic link, so a transfer can run without Wi-Fi or Bluetooth.

The modem is written in C++17 and uses miniaudio for audio input and output. The sender and receiver are Python command-line tools that call the modem through SWIG bindings.

## How it works

```mermaid
flowchart LR
    input["Input file"] --> sender["Sender<br/>Packetize, encode, modulate"]
    sender -->|"Data over sound"| receiver["Receiver<br/>Demodulate, decode, reassemble"]
    receiver --> output["Received file"]
    receiver -.->|"Acknowledgments over sound"| sender
```

1. **Handshake.** The sender announces the file name and size. The two sides synchronize their audio timing, and the receiver sends the transfer channel settings from [`apps/transfer.json`](apps/transfer.json).
2. **Transfer.** The sender splits the file into numbered segments and transmits them across configurable audio channels using an LTE like physical layer. Concatenated error-correcting codes help recover data from noisy signals, and the receiver sends acknowledgments in separate time slots so the sender can retry missing segments.
3. **Close.** Once every segment is acknowledged, the sender requests closure. The receiver saves the reconstructed file under its original name and confirms completion.

## Quick start

Build from the repository root on each computer. The Makefile supports macOS and Linux. You’ll need `make`, a C++17 compiler, Python 3 with development headers, and SWIG. If SWIG is missing, `make` uses Python’s `pip` to install it locally under `build/`.

```sh
make
```

Use two nearby computers with working speakers and microphones. Both computers need audio input and output because acknowledgments travel back as sound. Start the receiver first, then send a small file from the other computer.

```sh
# On the receiving computer
mkdir -p received
python3 apps/receiver.py received

# On the sending computer
python3 apps/sender.py path/to/file.txt
```

Both terminals show transfer progress and signal-to-noise estimates for their receive channels. Files can be up to 10 MiB.
