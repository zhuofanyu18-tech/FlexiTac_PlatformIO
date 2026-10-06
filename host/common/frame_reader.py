"""FlexiTac binary frame protocol: AA 55 + rows*cols uint8 (ADC10 >> 2)."""

import numpy as np

MAGIC = b"\xaa\x55"
COLS = 32
BAUD = 2_000_000


def default_mux_offset(rows):
    """Firmware default: 12-row builds scan MUX 4..15, 16-row builds scan 0..15."""
    return 4 if rows == 12 else 0


def check_mux_offset(parser, rows, offset):
    offset = default_mux_offset(rows) if offset is None else offset
    if not 0 <= offset <= 16 - rows:
        parser.error("rows + mux-offset must fit 16 MUX channels")
    return offset


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
