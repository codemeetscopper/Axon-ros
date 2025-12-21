#!/usr/bin/env python3
import time
from pathlib import Path

ALLOWLIST_PATH = Path(__file__).resolve().parent.parent / "scripts" / "gateway_allowlist.txt"


def load_allowlist() -> list[str]:
    if not ALLOWLIST_PATH.exists():
        return []
    return [line.strip() for line in ALLOWLIST_PATH.read_text().splitlines() if line.strip()]


def main() -> None:
    allowlist = load_allowlist()
    print("Axon gateway placeholder running with allowlist entries:")
    for entry in allowlist:
        print(f" - {entry}")
    while True:
        time.sleep(5)


if __name__ == "__main__":
    main()
