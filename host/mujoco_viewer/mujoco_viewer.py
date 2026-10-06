"""FlexiTac MuJoCo viewer: tactile frames as a continuous, height-colored terrain.

Serial frames are read in a background thread, baseline-calibrated with the
same pipeline as heatmap/heatmap.py, and rendered as a MuJoCo height field.
"""

import argparse
from pathlib import Path
import sys
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np
import serial

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.frame_reader import BAUD, COLS, FrameReader, check_mux_offset
from common.processing import TactileProcessor
from terrain import COLORMAPS, Terrain

# GLFW key codes used by MuJoCo's key_callback.
KEY_ENTER, KEY_EQUAL, KEY_MINUS, KEY_KP_ADD, KEY_KP_SUBTRACT = 257, 61, 45, 334, 333


class SerialSource:
    """Background reader; keeps only the newest processed 0..1 intensity."""

    def __init__(self, port, rows, baud, processor):
        self.processor = processor
        self.port = serial.Serial(port, baud, timeout=0.02, exclusive=True)
        self.reader = FrameReader(self.port, rows, COLS)
        self.lock = threading.Lock()
        self.intensity = np.zeros((rows, COLS), dtype=np.float32)
        self.raw = None
        self.frames = 0
        self.error = None
        self.recalibrate_requested = False
        self.running = True
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        last_frame = time.monotonic()
        try:
            while self.running:
                frame = self.reader.read()
                now = time.monotonic()
                if frame is None:
                    if now - last_frame > 8:
                        raise TimeoutError("No valid frames for 8 seconds. Check binary firmware, port, baud "
                                           f"and --rows. bytes_read={self.reader.bytes_read} "
                                           f"discarded={self.reader.discarded_bytes}")
                    continue
                last_frame = now
                if self.recalibrate_requested:
                    self.recalibrate_requested = False
                    self.processor.recalibrate()
                was_calibrated = self.processor.calibrated
                intensity = self.processor.update(frame)
                if self.processor.calibrated and not was_calibrated:
                    print("Calibration complete. Press the sensor.", flush=True)
                with self.lock:
                    self.raw = frame
                    self.frames += 1
                    if intensity is not None:
                        self.intensity = intensity
        except (serial.SerialException, OSError, TimeoutError) as error:
            self.error = error

    def latest(self):
        with self.lock:
            return self.intensity.copy()

    def status(self):
        with self.lock:
            raw = self.raw
        if raw is None:
            return "waiting for frames"
        r, c = np.unravel_index(raw.argmax(), raw.shape)
        state = "" if self.processor.calibrated else " (calibrating, keep unloaded)"
        return (f"raw_max={raw.max()} at ({r},{c}) scale={self.processor.scale:g} "
                f"sync_losses={self.reader.sync_losses}{state}")

    def recalibrate(self):
        self.recalibrate_requested = True
        print("Recalibrating: release the sensor.", flush=True)

    def close(self):
        self.running = False
        self.thread.join(timeout=1)
        self.port.close()


class DemoSource:
    """Synthetic presses for checking the viewer without hardware."""

    def __init__(self, rows, processor):
        self.rows = rows
        self.processor = processor
        self.frames = 0
        self.error = None
        self.start = time.monotonic()
        self.yy, self.xx = np.mgrid[0:rows, 0:COLS].astype(np.float32)

    def bump(self, row, col, sigma, peak):
        return peak * np.exp(-((self.xx - col) ** 2 + (self.yy - row) ** 2) / (2 * sigma ** 2))

    def latest(self):
        t = time.monotonic() - self.start
        self.frames += 1
        rows = self.rows - 1
        delta = (self.bump(rows * (0.5 + 0.3 * np.sin(t * 0.9)), 16 + 12 * np.sin(t * 0.6), 2.2,
                           60 * (0.6 + 0.4 * np.sin(t * 1.7)))
                 + self.bump(rows * 0.3, 24 + 4 * np.cos(t * 1.1), 1.6, 25 * (1 + np.sin(t * 2.3))))
        return self.processor.normalize(delta - self.processor.threshold, self.processor.scale)

    def status(self):
        return f"demo scale={self.processor.scale:g}"

    def recalibrate(self):
        print("Demo mode: nothing to calibrate.", flush=True)

    def close(self):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Linux /dev/ttyUSB*, macOS /dev/cu.*, Windows COM*")
    parser.add_argument("--demo", action="store_true", help="Synthetic presses; no serial port needed")
    parser.add_argument("--rows", type=int, choices=(12, 16), default=12)
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--mux-offset", type=int, default=None, help="Must match firmware; used for checks only")
    parser.add_argument("--threshold", type=float, default=4, help="Smoothed ADC delta below this stays flat")
    parser.add_argument("--scale", type=float, default=40, help="ADC delta at full height; smaller = more sensitive")
    parser.add_argument("--gamma", type=float, default=0.6, help="<1 lifts light touches; 1 = linear")
    parser.add_argument("--alpha", type=float, default=0.3, help="Temporal smoothing 0..1; larger = faster/noisier")
    parser.add_argument("--cmap", choices=COLORMAPS, default="turbo", help="Height color map")
    parser.add_argument("--height-mm", type=float, default=30, help="Height of a full-scale peak")
    parser.add_argument("--pitch-mm", type=float, default=5, help="Display spacing between sensor cells")
    parser.add_argument("--upsample", type=int, default=8, help="Terrain vertices per sensor cell")
    parser.add_argument("--blur", type=float, default=0.7, help="Gaussian sigma in sensor cells; 0 = cubic only")
    parser.add_argument("--fps", type=float, default=60, help="Render update rate limit")
    parser.add_argument("--snapshot", help="Render one offscreen PNG after --seconds and exit (no window)")
    parser.add_argument("--seconds", type=float, default=0, help="Stop after this many seconds; 0 until closed")
    args = parser.parse_args()
    if not args.demo and not args.port:
        parser.error("--port is required unless --demo is used")
    if (args.scale <= 0 or args.gamma <= 0 or args.threshold < 0 or not 0 < args.alpha <= 1
            or args.height_mm <= 0 or args.pitch_mm <= 0 or not 1 <= args.upsample <= 32
            or args.blur < 0 or args.fps <= 0 or args.seconds < 0):
        parser.error("invalid display parameter")
    check_mux_offset(parser, args.rows, args.mux_offset)

    terrain = Terrain(args.rows, COLS, args.pitch_mm / 1000, args.height_mm / 1000,
                      args.upsample, args.blur, args.cmap)
    processor = TactileProcessor(args.rows, COLS, args.threshold, args.scale, args.gamma, args.alpha)
    source = DemoSource(args.rows, processor) if args.demo else SerialSource(args.port, args.rows, args.baud, processor)
    if not args.demo:
        print("Keep the sensor unloaded for the first 30 frames.", flush=True)
    try:
        if args.snapshot:
            snapshot(args, terrain, source)
        else:
            run_viewer(args, terrain, processor, source)
    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        source.close()
    if source.error:
        raise SystemExit(str(source.error))


def snapshot(args, terrain, source):
    import cv2

    deadline = time.monotonic() + max(args.seconds, 1)
    while time.monotonic() < deadline and source.error is None:
        terrain.apply(source.latest())
        time.sleep(1 / args.fps)
    mujoco.mj_forward(terrain.model, terrain.data)
    with mujoco.Renderer(terrain.model, 720, 1280) as renderer:
        camera = mujoco.MjvCamera()
        terrain.set_camera(camera)
        renderer.update_scene(terrain.data, camera)
        cv2.imwrite(args.snapshot, renderer.render()[..., ::-1])
    print(f"Saved {args.snapshot}; {source.status()}")


def run_viewer(args, terrain, processor, source):
    def on_key(key):
        if key == KEY_ENTER:
            source.recalibrate()
        elif key in (KEY_EQUAL, KEY_KP_ADD):
            processor.adjust(0.8)
            print(f"More sensitive: scale={processor.scale:g}", flush=True)
        elif key in (KEY_MINUS, KEY_KP_SUBTRACT):
            processor.adjust(1.25)
            print(f"Less sensitive: scale={processor.scale:g}", flush=True)

    print("Mouse: left-drag rotate, right-drag pan, scroll zoom. "
          "Enter: recalibrate; =/+: more sensitive; -: less sensitive.", flush=True)
    before = set(threading.enumerate())
    try:
        with mujoco.viewer.launch_passive(terrain.model, terrain.data, key_callback=on_key,
                                          show_left_ui=False, show_right_ui=False) as viewer:
            with viewer.lock():
                terrain.set_camera(viewer.cam)
            start = last_status = time.monotonic()
            renders = 0
            while viewer.is_running() and source.error is None:
                tick = time.monotonic()
                with viewer.lock():
                    terrain.apply(source.latest())
                # These block until the render thread uploads; never call under viewer.lock().
                viewer.update_hfield(terrain.hfield_id)
                viewer.update_texture(terrain.texture_id)
                viewer.sync()
                renders += 1
                now = time.monotonic()
                if now - last_status >= 1:
                    print(f"render_fps={renders / (now - last_status):.1f} sensor_frames={source.frames} "
                          f"peak={terrain.peak:.2f} {source.status()}", flush=True)
                    renders, last_status = 0, now
                if args.seconds and now - start >= args.seconds:
                    break
                time.sleep(max(0, 1 / args.fps - (now - tick)))
    finally:
        # MuJoCo's render thread must stop before glfw.terminate() runs at exit,
        # otherwise Python segfaults on shutdown.
        for thread in set(threading.enumerate()) - before:
            thread.join(timeout=3)


if __name__ == "__main__":
    try:
        main()
    except (serial.SerialException, OSError) as error:
        raise SystemExit(str(error)) from error
