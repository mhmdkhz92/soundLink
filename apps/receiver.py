import argparse
import sys
from pathlib import Path
from time import monotonic, sleep

APP_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(APP_DIR.parent / "build"))

from utils import print_report, read_channels
from soundlink import Receiver


parser = argparse.ArgumentParser()
parser.add_argument("output_folder", type=Path)

output_folder = parser.parse_args().output_folder.resolve()
if not output_folder.is_dir():
    parser.error("output must be an existing folder")


channels = read_channels(APP_DIR / "transfer.json")
receiver = Receiver(str(output_folder))
receiver.set_tr_channels(channels["data"], channels["control"])
print(f"\n Receiving into: {output_folder}", flush=True)
receiver.start()
previous_lines = 0
next_report = monotonic()
while True:
    receiver.step()
    finished = receiver.is_finished()
    now = monotonic()
    if finished or now >= next_report:
        previous_lines = print_report(receiver.get_report(), "Received", previous_lines)
        next_report = now + 0.25
    if finished:
        break
    sleep(0.01)
print("Receiving finished")
