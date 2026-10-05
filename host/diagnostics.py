"""Raw uint8 statistics and recordings, independent of any GUI or calibration."""

import csv
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np


def frame_stats(frame):
    row, col = np.unravel_index(np.argmax(frame), frame.shape)
    return {
        "raw_min": int(frame.min()), "raw_max": int(frame.max()),
        "max_row": int(row), "max_col": int(col),
        "nonzero": int(np.count_nonzero(frame)), "mean": float(frame.mean()),
        "row_max": frame.max(axis=1), "row_mean": frame.mean(axis=1),
        "row_nonzero": np.count_nonzero(frame, axis=1),
        "col_max": frame.max(axis=0), "col_mean": frame.mean(axis=0),
        "col_nonzero": np.count_nonzero(frame, axis=0),
    }


class RawDiagnostics:
    """Collect every frame's metrics; print summaries at most once per second.

    With a record prefix, CSV is streamed and raw matrices are saved as NPZ on
    close. Use finite --seconds for recordings; NPZ matrices are held in RAM.
    """

    def __init__(self, rows, cols, mux_offset, record=None, label="unknown", metadata=None):
        self.rows, self.cols, self.mux_offset = rows, cols, mux_offset
        self.label = label
        self.count = self.window_count = 0
        self.started = self.last_report = None
        self.window_max = self.window_nonzero = 0
        self.window_peak = None
        self.row_max = np.zeros(rows, dtype=np.uint8)
        self.col_max = np.zeros(cols, dtype=np.uint8)
        self.latest = None
        self.frames, self.times, self.labels = [], [], []
        self.prefix = Path(record) if record else None
        self.csv_file = None
        self.metadata = dict(metadata or {})
        self.metadata.update(rows=rows, cols=cols, mux_offset=mux_offset,
                             units="uint8 = ADC10 >> 2; no baseline/threshold/filter",
                             coordinates="zero-based scan row/column; mux=row+offset",
                             created_utc=datetime.now(timezone.utc).isoformat())
        if self.prefix:
            self.prefix.parent.mkdir(parents=True, exist_ok=True)
            for suffix in (".npz", ".csv", ".json"):
                if Path(str(self.prefix) + suffix).exists():
                    raise FileExistsError(f"Recording exists: {self.prefix}{suffix}")
            self.csv_file = Path(str(self.prefix) + ".csv").open("x", newline="")
            scalar_names = ["raw_min", "raw_max", "max_row", "max_col", "nonzero", "mean"]
            self.array_names = ["row_max", "row_mean", "row_nonzero", "col_max", "col_mean", "col_nonzero"]
            names = ["frame", "elapsed_s", "label", *scalar_names]
            for name in self.array_names:
                names.extend(f"{name}_{i}" for i in range(rows if name.startswith("row") else cols))
            self.writer = csv.writer(self.csv_file)
            self.writer.writerow(names)

    def update(self, frame, now, reader=None):
        if self.started is None:
            self.started = self.last_report = now
        stats = frame_stats(frame)
        self.count += 1
        self.window_count += 1
        if self.window_peak is None or stats["raw_max"] > self.window_max:
            self.window_peak = (stats["max_row"], stats["max_col"])
        self.window_max = max(self.window_max, stats["raw_max"])
        self.window_nonzero = max(self.window_nonzero, stats["nonzero"])
        np.maximum(self.row_max, stats["row_max"], out=self.row_max)
        np.maximum(self.col_max, stats["col_max"], out=self.col_max)
        self.latest = stats
        if self.csv_file:
            elapsed = now - self.started
            self.frames.append(frame.copy())
            self.times.append(elapsed)
            self.labels.append(self.label)
            values = [self.count, f"{elapsed:.6f}", self.label]
            values.extend(stats[name] for name in ("raw_min", "raw_max", "max_row", "max_col", "nonzero", "mean"))
            for name in self.array_names:
                values.extend(stats[name].tolist())
            self.writer.writerow(values)
        if now - self.last_report >= 1:
            self.report(now, reader)

    def report(self, now, reader=None):
        if not self.window_count:
            return
        stats = self.latest
        row, col = self.window_peak
        elapsed = max(now - self.last_report, 1e-9)
        print(f"frames={self.count} fps={self.window_count / elapsed:.1f} label={self.label} "
              f"raw_min={stats['raw_min']} raw_max={stats['raw_max']} "
              f"max_at=({stats['max_row']},{stats['max_col']}) "
              f"mux={stats['max_row'] + self.mux_offset} nonzero={stats['nonzero']}/{self.rows*self.cols} "
              f"mean={stats['mean']:.3f} window_max={self.window_max}@({row},{col}) "
              f"window_nonzero_max={self.window_nonzero}", flush=True)
        print(f"  row_max_window[mux {self.mux_offset}..{self.mux_offset+self.rows-1}]={self.row_max.tolist()} "
              f"row_mean_last={np.round(stats['row_mean'], 2).tolist()} row_nonzero_last={stats['row_nonzero'].tolist()}")
        print(f"  col_max_window[scan 0..{self.cols-1}]={self.col_max.tolist()} "
              f"col_mean_last={np.round(stats['col_mean'], 2).tolist()} col_nonzero_last={stats['col_nonzero'].tolist()}")
        if reader is not None:
            print(f"  bytes_read={reader.bytes_read} discarded_bytes={reader.discarded_bytes} sync_losses={reader.sync_losses}")
        self.window_count = self.window_max = self.window_nonzero = 0
        self.window_peak = None
        self.row_max.fill(0)
        self.col_max.fill(0)
        self.last_report = now
        if self.csv_file:
            self.csv_file.flush()

    def close(self, reader=None):
        if not self.csv_file:
            return
        self.csv_file.close()
        self.csv_file = None
        frames = np.stack(self.frames) if self.frames else np.empty((0, self.rows, self.cols), dtype=np.uint8)
        np.savez_compressed(str(self.prefix) + ".npz", frames=frames,
                            elapsed_s=np.asarray(self.times), labels=np.asarray(self.labels, dtype="U64"),
                            mux_offset=np.int64(self.mux_offset))
        self.metadata["frames"] = self.count
        if reader is not None:
            self.metadata.update(bytes_read=reader.bytes_read, discarded_bytes=reader.discarded_bytes,
                                 sync_losses=reader.sync_losses)
        Path(str(self.prefix) + ".json").write_text(json.dumps(self.metadata, indent=2, ensure_ascii=False) + "\n")
        print(f"Saved {self.count} raw frames: {self.prefix}.npz (+ .csv, .json)")
