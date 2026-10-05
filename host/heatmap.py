"""FlexiTac binary heatmap. Match --rows to the firmware environment."""

import argparse
import os
from pathlib import Path
import time

import numpy as np
import serial

from diagnostics import RawDiagnostics

MAGIC = b"\xaa\x55"


class FrameReader:
    """Keep partial reads; validate the next header at EVERY frame boundary.

    One frame of lookahead avoids silently accepting a packet with lost bytes.
    The legacy protocol has no CRC/sequence/geometry field, so this cannot prove
    payload integrity or distinguish every possible false marker collision.
    """

    def __init__(self, port, rows, cols):
        self.port = port
        self.rows = rows
        self.cols = cols
        self.size = rows * cols
        self.buffer = bytearray()
        self.synced = False
        self.bytes_read = self.discarded_bytes = self.sync_losses = 0

    def discard(self, count):
        if count:
            if self.synced:
                self.sync_losses += 1
            self.synced = False
            self.discarded_bytes += count
            del self.buffer[:count]

    def read(self):
        packet = self.size + len(MAGIC)
        while True:
            index = self.buffer.find(MAGIC)
            if index < 0:
                # Preserve only a possible partial marker, not an arbitrary byte.
                keep = int(self.buffer.endswith(MAGIC[:1]))
                self.discard(len(self.buffer) - keep)
                break
            if index:
                self.discard(index)
            if len(self.buffer) < packet + len(MAGIC):
                break
            if self.buffer[packet:packet + 2] != MAGIC:
                self.discard(1)
                continue
            raw = bytes(self.buffer[2:packet])
            del self.buffer[:packet]
            self.synced = True
            return np.frombuffer(raw, dtype=np.uint8).reshape(self.rows, self.cols)
        chunk = self.port.read(max(1, min(self.port.in_waiting, 8192)))
        self.bytes_read += len(chunk)
        self.buffer.extend(chunk)
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Linux /dev/ttyUSB*, macOS /dev/cu.*, Windows COM*")
    parser.add_argument("--rows", type=int, choices=(12, 16), default=12)
    parser.add_argument("--cols", type=int, default=32, choices=(32,))
    parser.add_argument("--baud", type=int, default=2_000_000)
    parser.add_argument("--mux-offset", type=int, default=None, help="Metadata/row labels only; cannot change firmware")
    parser.add_argument("--threshold", type=float, default=15)
    parser.add_argument("--scale", type=float, default=100, help="Fixed ADC delta display range")
    parser.add_argument("--raw", action="store_true", help="Unfiltered raw matrix; skip baseline calibration")
    parser.add_argument("--raw-scale", type=float, default=255, help="Fixed raw display ceiling (ADC8 units); data unchanged")
    parser.add_argument("--diagnostics", action="store_true", help="Detailed per-second row/column raw statistics")
    parser.add_argument("--record", help="New raw recording prefix (.npz + per-frame .csv + .json)")
    parser.add_argument("--label", default="unknown", help="Recording condition; keys 1/2/3 change condition")
    parser.add_argument("--firmware", default="unknown", help="Environment name for recording metadata")
    parser.add_argument("--seconds", type=float, default=0, help="Stop after this many seconds of frames; 0 until Q")
    args = parser.parse_args()
    if args.scale <= 0 or args.raw_scale <= 0 or args.threshold < 0 or args.seconds < 0 or args.baud <= 0:
        parser.error("scales/baud must be positive; threshold/seconds nonnegative")
    offset = args.mux_offset if args.mux_offset is not None else (4 if args.rows == 12 else 0)
    if not 0 <= offset <= 16 - args.rows:
        parser.error("rows + mux-offset must fit 16 MUX channels")
    # Headless diagnostics import FrameReader without loading Qt/OpenCV.
    requested_font_dir = os.environ.get("QT_QPA_FONTDIR")
    import cv2

    # Some OpenCV wheels overwrite QT_QPA_FONTDIR with a missing qt/fonts.
    # Restore a valid user choice or choose an existing Linux font directory.
    if requested_font_dir and Path(requested_font_dir).is_dir():
        os.environ["QT_QPA_FONTDIR"] = requested_font_dir
    elif "QT_QPA_FONTDIR" in os.environ and not Path(os.environ["QT_QPA_FONTDIR"]).is_dir():
        for font_dir in ("/usr/share/fonts/truetype/dejavu", "/usr/share/fonts/truetype/liberation2"):
            if Path(font_dir).is_dir():
                os.environ["QT_QPA_FONTDIR"] = font_dir
                print(f"Qt fonts: {font_dir} (GUI font path only; ADC data unchanged).")
                break

    calibration = []
    baseline = None
    filtered = np.zeros((args.rows, args.cols), dtype=np.float32)
    count = 0
    last_status = last_frame = time.monotonic()
    start = None
    diag = reader = None
    print(f"Binary frame: {args.rows}x{args.cols}, {2+args.rows*args.cols} bytes; "
          f"scan rows map to MUX {offset}..{offset+args.rows-1}; coordinates are zero-based.")
    if args.raw:
        print(f"RAW: no baseline or time filter, fixed display 0..{args.raw_scale:g} ADC8. "
              "Threshold/scale apply to delta mode only.")
    else:
        print("Keep the sensor unloaded for the first 30 frames. R: recalibrate.")
    print("Q/Esc: quit; 1: released; 2: pressed; 3: moving (labels only).")
    try:
        if args.diagnostics or args.record:
            diag = RawDiagnostics(args.rows, args.cols, offset, args.record, args.label, vars(args))
        cv2.namedWindow("FlexiTac", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("FlexiTac", args.cols * 25, args.rows * 25)
        with serial.Serial(args.port, args.baud, timeout=0.02, exclusive=True) as port:
            reader = FrameReader(port, args.rows, args.cols)
            last_frame = time.monotonic()
            while True:
                frame = reader.read()
                now = time.monotonic()
                if frame is not None:
                    last_frame = now
                    if start is None:
                        start = last_status = now
                    count += 1
                    if diag:
                        diag.update(frame, now, reader)
                    values = frame.astype(np.float32)
                    if not args.raw and baseline is None:
                        calibration.append(values)
                        if len(calibration) == 30:
                            baseline = np.median(np.stack(calibration), axis=0)
                            calibration.clear()
                            print("Calibration complete.")
                    if args.raw:
                        display = np.clip(values / args.raw_scale, 0, 1)
                    elif baseline is not None:
                        normalized = np.clip((values - baseline - args.threshold) / args.scale, 0, 1)
                        filtered = 0.2 * normalized + 0.8 * filtered
                        display = filtered
                    else:
                        display = None
                    if display is not None:
                        colors = cv2.applyColorMap((display * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
                        # Nearest-neighbour enlarged cells preserve the raw matrix layout.
                        colors = cv2.resize(colors, (args.cols*25, args.rows*25), interpolation=cv2.INTER_NEAREST)
                        cv2.imshow("FlexiTac", colors)
                    if diag is None and now - last_status >= 1:
                        r, c = np.unravel_index(frame.argmax(), frame.shape)
                        print(f"fps_avg={count / max(now-start, 1e-9):.1f} raw_min={frame.min()} raw_max={frame.max()} "
                              f"max_at=({r},{c}) mux={r+offset} nonzero={np.count_nonzero(frame)} "
                              f"sync_losses={reader.sync_losses} discarded_bytes={reader.discarded_bytes}")
                        last_status = now
                    if args.seconds and now - start >= args.seconds:
                        break
                elif now - last_frame > 8:
                    raise TimeoutError("No valid frames for 8 seconds. Check binary firmware, port, baud and --rows. "
                                       f"bytes_read={reader.bytes_read} discarded={reader.discarded_bytes}")
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("r") and not args.raw:
                    baseline = None
                    calibration.clear()
                    filtered.fill(0)
                    print("Recalibrating: release the sensor.")
                if key in (ord("1"), ord("2"), ord("3")):
                    label = {ord("1"): "released", ord("2"): "pressed", ord("3"): "moving"}[key]
                    if diag:
                        diag.label = label
                    print(f"Condition label={label}; annotation only.")
    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        if diag:
            diag.report(time.monotonic(), reader)
            diag.close(reader)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except (serial.SerialException, TimeoutError, OSError) as error:
        raise SystemExit(str(error)) from error
