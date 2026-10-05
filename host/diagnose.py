"""Headless binary raw diagnostic recorder and release/press comparison."""

import argparse
import time

import numpy as np
import serial

from diagnostics import RawDiagnostics
from heatmap import FrameReader


def load_recording(path):
    with np.load(path, allow_pickle=False) as data:
        frames = data["frames"]
        offset = int(data["mux_offset"])
    if frames.ndim != 3 or frames.shape[0] == 0 or frames.dtype != np.uint8:
        raise ValueError(f"{path}: expected nonempty uint8 (frames, rows, cols)")
    return frames.astype(np.float32), offset


def compare(released_path, pressed_path):
    released, offset = load_recording(released_path)
    pressed, pressed_offset = load_recording(pressed_path)
    if released.shape[1:] != pressed.shape[1:] or offset != pressed_offset:
        raise ValueError("Recording geometry/MUX offset differs; compare the same firmware and wiring")
    rest = np.median(released, axis=0)
    contact = np.median(pressed, axis=0)
    delta = contact - rest  # signed; never clip negative changes
    # A descriptive spread, not a claim of a statistically calibrated threshold.
    noise_span = np.percentile(released, 95, axis=0) - np.percentile(released, 5, axis=0)
    row, col = np.unravel_index(np.argmax(np.abs(delta)), delta.shape)
    print(f"released_frames={len(released)} pressed_frames={len(pressed)} "
          f"released_max={int(released.max())} pressed_max={int(pressed.max())}")
    print(f"largest_median_change={delta[row,col]:+.2f} at row={row} mux={row+offset} col={col}; "
          f"release_p95-p05_here={noise_span[row,col]:.2f}")
    print("signed median delta matrix (uint8 ADC units, no threshold):")
    print(np.array2string(delta, precision=1, suppress_small=True, threshold=delta.size, max_line_width=180))
    print("row_delta_mean=", np.round(delta.mean(axis=1), 3).tolist())
    print("col_delta_mean=", np.round(delta.mean(axis=0), 3).tolist())
    print("Largest absolute median changes (compare their spread and touch location):")
    for index in np.argsort(np.abs(delta).ravel())[-10:][::-1]:
        r, c = np.unravel_index(index, delta.shape)
        print(f"  mux={r+offset} col={c} rest={rest[r,c]:.1f} press={contact[r,c]:.1f} "
              f"delta={delta[r,c]:+.1f} release_p95-p05={noise_span[r,c]:.1f}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compare", nargs=2, metavar=("RELEASED_NPZ", "PRESSED_NPZ"))
    parser.add_argument("--port")
    parser.add_argument("--rows", type=int, choices=(12, 16), default=16)
    parser.add_argument("--baud", type=int, default=2_000_000)
    parser.add_argument("--mux-offset", type=int, default=None)
    parser.add_argument("--seconds", type=float, default=10, help="Record duration after first frame; 0 until Ctrl-C")
    parser.add_argument("--warmup", type=float, default=2, help="Ignore data during Nano reset and analog startup")
    parser.add_argument("--record", help="New output prefix for .npz, .csv and .json")
    parser.add_argument("--label", default="unknown", help="released / pressed / moving / other")
    parser.add_argument("--firmware", default="unknown", help="Environment name for recording metadata")
    args = parser.parse_args()
    if args.compare:
        compare(*args.compare)
        return
    if not args.port:
        parser.error("--port is required for live capture")
    if args.seconds < 0 or args.warmup < 0 or args.baud <= 0:
        parser.error("seconds/warmup must be nonnegative and baud positive")
    offset = args.mux_offset if args.mux_offset is not None else (4 if args.rows == 12 else 0)
    if not 0 <= offset <= 16 - args.rows:
        parser.error("rows + mux-offset must fit 16 MUX channels")
    reader = None
    diag = RawDiagnostics(args.rows, 32, offset, args.record, args.label, vars(args))
    print(f"BINARY {args.rows}x32; frame={2+args.rows*32} bytes; mux={offset}..{offset+args.rows-1}; "
          f"label={args.label}. No GUI/calibration. Ctrl-C stops and saves.", flush=True)
    try:
        with serial.Serial(args.port, args.baud, timeout=0.02, exclusive=True) as port:
            reader = FrameReader(port, args.rows, 32)
            opened = last_frame = time.monotonic()
            capturing = False
            while True:
                frame = reader.read()
                now = time.monotonic()
                if frame is not None:
                    last_frame = now
                    if now - opened < args.warmup:
                        continue
                    if not capturing:
                        print("Capture started: maintain the labelled condition.", flush=True)
                        capturing = True
                    diag.update(frame, now, reader)
                    if args.seconds and now - diag.started >= args.seconds:
                        break
                elif now - last_frame > 8:
                    raise TimeoutError("No valid frames for 8 s; check binary firmware, --rows, baud and port. "
                                       f"bytes_read={reader.bytes_read}, discarded={reader.discarded_bytes}")
    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        diag.report(time.monotonic(), reader)
        diag.close(reader)


if __name__ == "__main__":
    try:
        main()
    except (serial.SerialException, TimeoutError, OSError, ValueError) as error:
        raise SystemExit(str(error)) from error
