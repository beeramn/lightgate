import csv
import sys
from pathlib import Path

import serial


BAUD = 115200


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 log_serial_csv.py <serial_port> [baud]")
        print("Example macOS: python3 log_serial_csv.py /dev/tty.usbmodem2101")
        print("Example Windows: python log_serial_csv.py COM3")
        sys.exit(1)

    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) >= 3 else BAUD

    script_dir = Path(__file__).resolve().parent
    csv_path = script_dir / "speeds.csv"

    file_exists = csv_path.exists() and csv_path.stat().st_size > 0

    with serial.Serial(port, baud, timeout=1) as ser, open(csv_path, "a", newline="") as f:
        writer = csv.writer(f)

        if not file_exists:
            writer.writerow([
                "event",
                "session_id",
                "seq",
                "rx_time_us",
                "trigger_time_us",
                "dt_us",
                "speed_mps",
                "speed_kmh",
            ])
            f.flush()

        print(f"Listening on {port} @ {baud}")
        print(f"Saving CSV to: {csv_path}")
        print("Press Ctrl+C to stop.\n")

        try:
            while True:
                line = ser.readline().decode(errors="ignore").strip()

                if not line:
                    continue

                print(line)

                if line.startswith("CSV_HEADER,"):
                    continue

                if not line.startswith("CSV,"):
                    continue

                row = line.split(",")[1:]

                if len(row) != 8:
                    print(f"Skipping malformed CSV row: {line}")
                    continue

                writer.writerow(row)
                f.flush()

        except KeyboardInterrupt:
            print("\nStopped logging.")


if __name__ == "__main__":
    main()