import argparse
import sys
from pathlib import Path
from time import monotonic, sleep
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "build"))

from utils import print_report
from soundlink import Sender

parser = argparse.ArgumentParser()
parser.add_argument("input_file", type=Path)
source = parser.parse_args().input_file.resolve()
if not source.is_file():
    parser.error("input must be an existing file")



sender = Sender(str(source))
print(f"\n Sending: {source}", flush=True)
sender.start()
previous_lines = 0
next_report = monotonic()
while True:
    sender.step()
    finished = sender.is_finished()
    now = monotonic()
    if finished or now >= next_report:
        previous_lines = print_report(sender.get_report(), "Acknowledged", previous_lines)
        next_report = now + 0.25
    if finished:
        break
    sleep(0.01)
print("Sending finished")
