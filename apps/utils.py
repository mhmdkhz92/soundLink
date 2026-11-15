import json
import sys

import soundlink as sl


MODULATIONS = {sl.BPSK: "BPSK", sl.QPSK: "QPSK", sl.QAM16: "16QAM", sl.QAM64: "64QAM"}
CODE_RATES = {sl.FEC12: "1/2", sl.FEC23: "2/3", sl.FEC34: "3/4", sl.FEC56: "5/6", sl.FEC78: "7/8"}


def print_report(report, progress_label, previous_lines=0):
    done, total = report.bytes_done, report.total_bytes
    width = 24
    filled = width * done // total if total else width
    bar = "#" * filled + "-" * (width - filled)
    lines = [
        f"State: {report.state}",
        f"{progress_label}  [{bar}]  {done} / {total} B  ({total - done} B left)",
    ]

    for number, channel in enumerate(report.rx, start=1):
        cfg = channel.config
        snr = f"{channel.snr_db:.1f}dB" if channel.snr_valid else "-"
        label = f"RX CH{number}"
        lines.append(
            f"{label:<7} {channel.name:<6} {cfg.fc:>2}kHz"
            f"  BW {cfg.bandwidth / 1000:>3g}kHz"
            f"  {MODULATIONS[cfg.mod]:<5} FEC {CODE_RATES[cfg.cr]}"
            f"  SNR {snr:>8}  {channel.received_bytes:>8}B"
        )

    if previous_lines and sys.stdout.isatty():
        print(f"\x1b[{previous_lines}A\r\x1b[J", end="")
    print("\n".join(lines), flush=True)
    return len(lines)


def read_channels(path):
    config = json.loads(path.read_text(encoding="utf-8"))
    bandwidths = {1.4: sl.BW1_4, 3: sl.BW3, 5: sl.BW5, 10: sl.BW10}
    code_rates = {name: value for value, name in CODE_RATES.items()}
    modulations = {name: value for value, name in MODULATIONS.items()}

    channels = {}
    for group in ("data", "control"):
        channels[group] = []
        for settings in config[group]:
            channel = sl.ChannelInfo(
                settings["center_khz"],
                code_rates[settings["code_rate"]],
                bandwidths[settings["bandwidth_khz"]],
                modulations[settings["modulation"]],
            )
            channels[group].append(channel)
    return channels
