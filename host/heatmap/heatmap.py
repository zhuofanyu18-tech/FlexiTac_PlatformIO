"""FlexiTac binary heatmap. Match --rows to the firmware environment."""

import argparse
import os
from pathlib import Path
import sys
import time

import numpy as np
import serial

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.diagnostics import RawDiagnostics
from common.frame_reader import BAUD, COLS, FrameReader, check_mux_offset
from common.processing import TactileProcessor

CELL = 25
COLORMAPS = ("viridis", "turbo", "inferno", "jet", "hot")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Linux /dev/ttyUSB*, macOS /dev/cu.*, Windows COM*")
    parser.add_argument("--rows", type=int, choices=(12, 16), default=12)
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--mux-offset", type=int, default=None, help="Metadata/row labels only; cannot change firmware")
    parser.add_argument("--threshold", type=float, default=4, help="Smoothed ADC delta below this stays dark")
    parser.add_argument("--scale", type=float, default=40, help="ADC delta shown at full brightness; smaller = brighter")
    parser.add_argument("--gamma", type=float, default=0.6, help="<1 lifts light touches; 1 = linear")
    parser.add_argument("--alpha", type=float, default=0.3, help="Temporal smoothing 0..1; larger = faster/noisier")
    parser.add_argument("--cmap", choices=COLORMAPS, default="viridis")
    parser.add_argument("--raw", action="store_true", help="Unfiltered raw matrix; skip baseline calibration")
    parser.add_argument("--raw-scale", type=float, default=64, help="Raw value shown at full brightness (ADC8 units)")
    parser.add_argument("--diagnostics", action="store_true", help="Detailed per-second row/column raw statistics")
    parser.add_argument("--record", help="New raw recording prefix (.npz + per-frame .csv + .json)")
    parser.add_argument("--label", default="unknown", help="Recording condition; keys 1/2/3 change condition")
    parser.add_argument("--firmware", default="unknown", help="Environment name for recording metadata")
    parser.add_argument("--seconds", type=float, default=0, help="Stop after this many seconds of frames; 0 until Q")
    args = parser.parse_args()
    if (args.scale <= 0 or args.raw_scale <= 0 or args.gamma <= 0 or args.threshold < 0
            or args.seconds < 0 or args.baud <= 0 or not 0 < args.alpha <= 1):
        parser.error("scales/gamma/baud must be positive, alpha in (0, 1], threshold/seconds nonnegative")
    offset = check_mux_offset(parser, args.rows, args.mux_offset)
    # Headless diagnostics import common/ without loading Qt/OpenCV.
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

    colormap = getattr(cv2, f"COLORMAP_{args.cmap.upper()}")
    processor = TactileProcessor(args.rows, COLS, args.threshold, args.scale, args.gamma, args.alpha)
    count = 0
    last_status = last_frame = time.monotonic()
    start = None
    diag = reader = None
    print(f"Binary frame: {args.rows}x{COLS}, {2+args.rows*COLS} bytes; "
          f"scan rows map to MUX {offset}..{offset+args.rows-1}; coordinates are zero-based.")
    if args.raw:
        print(f"RAW: no baseline or time filter, full brightness at {args.raw_scale:g} ADC8.")
    else:
        print("Keep the sensor unloaded for the first 30 frames. R: recalibrate.")
    print("+/-: brighter/darker; Q/Esc: quit; 1: released; 2: pressed; 3: moving (labels only).")
    try:
        if args.diagnostics or args.record:
            diag = RawDiagnostics(args.rows, COLS, offset, args.record, args.label, vars(args))
        cv2.namedWindow("FlexiTac", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("FlexiTac", COLS * CELL, args.rows * CELL)
        with serial.Serial(args.port, args.baud, timeout=0.02, exclusive=True) as port:
            reader = FrameReader(port, args.rows, COLS)
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
                    if args.raw:
                        display = processor.normalize(frame.astype(np.float32), args.raw_scale)
                    else:
                        was_calibrated = processor.calibrated
                        display = processor.update(frame)
                        if processor.calibrated and not was_calibrated:
                            print("Calibration complete.")
                    if display is not None:
                        colors = cv2.applyColorMap((display * 255).astype(np.uint8), colormap)
                        # Nearest-neighbour enlarged cells preserve the raw matrix layout.
                        colors = cv2.resize(colors, (COLS*CELL, args.rows*CELL), interpolation=cv2.INTER_NEAREST)
                        cv2.imshow("FlexiTac", colors)
                    if diag is None and now - last_status >= 1:
                        r, c = np.unravel_index(frame.argmax(), frame.shape)
                        print(f"fps_avg={count / max(now-start, 1e-9):.1f} raw_min={frame.min()} raw_max={frame.max()} "
                              f"max_at=({r},{c}) mux={r+offset} nonzero={np.count_nonzero(frame)} "
                              f"scale={processor.scale:g} "
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
                    processor.recalibrate()
                    print("Recalibrating: release the sensor.")
                if key in (ord("+"), ord("=")):
                    processor.adjust(0.8)
                    args.raw_scale = max(2, args.raw_scale * 0.8)
                    print(f"Brighter: scale={processor.scale:g} raw_scale={args.raw_scale:g}")
                if key in (ord("-"), ord("_")):
                    processor.adjust(1.25)
                    args.raw_scale = min(255, args.raw_scale * 1.25)
                    print(f"Darker: scale={processor.scale:g} raw_scale={args.raw_scale:g}")
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
