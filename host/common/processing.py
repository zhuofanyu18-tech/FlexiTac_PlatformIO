"""Baseline calibration, temporal smoothing and display normalization.

Shared by the OpenCV heatmap and the MuJoCo viewer so both show the same
numbers. Output is a 0..1 display intensity, not a calibrated force.
"""

import numpy as np


class TactileProcessor:
    """Per-frame pipeline: delta = raw - baseline -> EMA -> threshold/scale -> gamma.

    Smoothing happens BEFORE thresholding, so isolated single-frame ADC spikes
    are averaged down instead of flashing through the threshold.
    """

    def __init__(self, rows, cols, threshold=4.0, scale=40.0, gamma=0.6, alpha=0.3, calib_frames=30):
        self.shape = (rows, cols)
        self.threshold = threshold
        self.scale = scale
        self.gamma = gamma
        self.alpha = alpha
        self.calib_frames = calib_frames
        self.recalibrate()

    @property
    def calibrated(self):
        return self.baseline is not None

    def recalibrate(self):
        self.baseline = None
        self._calibration = []
        self.smoothed = np.zeros(self.shape, dtype=np.float32)

    def adjust(self, factor):
        """Brightness: a smaller full-scale value makes the same press brighter."""
        self.scale = float(np.clip(self.scale * factor, 2, 255))

    def update(self, frame):
        """Return 0..1 intensity, or None while collecting the baseline."""
        values = frame.astype(np.float32)
        if self.baseline is None:
            self._calibration.append(values)
            if len(self._calibration) < self.calib_frames:
                return None
            self.baseline = np.median(np.stack(self._calibration), axis=0)
            self._calibration.clear()
        delta = values - self.baseline
        self.smoothed += self.alpha * (delta - self.smoothed)
        return self.normalize(self.smoothed - self.threshold, self.scale)

    def normalize(self, values, scale):
        return np.clip(values / scale, 0, 1) ** self.gamma
