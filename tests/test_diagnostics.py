"""Protocol corruption, raw recording fidelity and signed comparison checks."""

from contextlib import redirect_stdout
import csv
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'host'))
from common.frame_reader import FrameReader, MAGIC
from common.diagnostics import RawDiagnostics, frame_stats
from diagnose.diagnose import compare


class FakePort:
    def __init__(self, data, chunk):
        self.data = bytearray(data)
        self.chunk = chunk

    @property
    def in_waiting(self):
        return min(len(self.data), self.chunk)

    def read(self, size):
        count = min(size, self.chunk, len(self.data))
        data = bytes(self.data[:count])
        del self.data[:count]
        return data


def decode(data, rows=12, chunk=7):
    port = FakePort(data, chunk)
    reader = FrameReader(port, rows, 32)
    frames = []
    for _ in range(len(data) * 2 + 10):
        had_input = bool(port.data)
        frame = reader.read()
        if frame is not None:
            frames.append(frame)
        elif not had_input:
            break
    return frames, reader


class ProtocolTests(unittest.TestCase):
    def test_partial_reads_and_headers_inside_payload(self):
        for rows in (12, 16):
            frames = [np.random.default_rng(i).integers(0, 256, (rows, 32), dtype=np.uint8) for i in range(3)]
            frames[0][2, 10:12] = [0xAA, 0x55]
            stream = b'garbage\xaa' + b''.join(MAGIC + f.tobytes() for f in frames) + MAGIC
            for chunk in (1, 7, 8192):
                with self.subTest(rows=rows, chunk=chunk):
                    actual, reader = decode(stream, rows, chunk)
                    self.assertEqual(len(actual), 3)
                    for result, expected in zip(actual, frames):
                        np.testing.assert_array_equal(result, expected)
                    self.assertEqual(reader.sync_losses, 0)
                    self.assertEqual(reader.discarded_bytes, 8)

    def test_lost_and_inserted_payload_bytes_reacquire_boundary(self):
        frames = [np.full((12, 32), i+1, dtype=np.uint8) for i in range(4)]
        for corrupt in (frames[1].tobytes()[:-1], frames[1].tobytes() + b'\x99'):
            for chunk in (1, 7, 8192):
                stream = MAGIC + frames[0].tobytes() + MAGIC + corrupt
                stream += MAGIC + frames[2].tobytes() + MAGIC + frames[3].tobytes() + MAGIC
                actual, reader = decode(stream, chunk=chunk)
                self.assertEqual(len(actual), 3)
                for result, expected in zip(actual, [frames[0], frames[2], frames[3]]):
                    np.testing.assert_array_equal(result, expected)
                self.assertEqual(reader.sync_losses, 1)

    def test_geometry_mismatch_does_not_accept_zero_stream(self):
        stream = (MAGIC + bytes(16*32))*5 + MAGIC
        actual, reader = decode(stream, rows=12)
        self.assertEqual(actual, [])
        self.assertGreater(reader.discarded_bytes, 0)

    def test_false_marker_and_partial_tail(self):
        expected = np.zeros((16, 32), dtype=np.uint8)
        expected[15, 31] = 7
        stream = MAGIC + b'noise' + MAGIC + expected.tobytes() + MAGIC + b'\x99'
        actual, _ = decode(stream, rows=16)
        self.assertEqual(len(actual), 1)
        np.testing.assert_array_equal(actual[0], expected)


class RecordingTests(unittest.TestCase):
    def test_statistics_and_files_preserve_low_raw_values(self):
        frame = np.zeros((16, 32), dtype=np.uint8)
        frame[3, 31] = 7
        frame[9, 1] = 3
        stats = frame_stats(frame)
        self.assertEqual((stats['raw_max'], stats['max_row'], stats['max_col'], stats['nonzero']), (7, 3, 31, 2))
        self.assertEqual(stats['row_mean'][3], 7/32)
        self.assertEqual(stats['col_nonzero'][1], 1)
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            prefix = Path(directory)/'raw'
            diag = RawDiagnostics(16, 32, 0, prefix, 'released')
            diag.update(frame, 100)
            diag.label = 'pressed'
            diag.update(frame + 1, 101)
            diag.close()
            with np.load(str(prefix)+'.npz', allow_pickle=False) as data:
                np.testing.assert_array_equal(data['frames'], [frame, frame+1])
                np.testing.assert_array_equal(data['elapsed_s'], [0, 1])
                self.assertEqual(data['labels'].tolist(), ['released', 'pressed'])
            with open(str(prefix)+'.csv') as file:
                records = list(csv.DictReader(file))
            self.assertEqual(records[0]['max_col'], '31')
            self.assertEqual(records[0]['row_max_3'], '7')
            self.assertEqual(records[0]['col_nonzero_1'], '1')
            self.assertEqual(json.loads(Path(str(prefix)+'.json').read_text())['frames'], 2)
            with self.assertRaises(FileExistsError):
                RawDiagnostics(16, 32, 0, prefix)

    def test_compare_keeps_negative_change_and_rejects_wrong_mapping(self):
        with tempfile.TemporaryDirectory() as directory:
            rest = np.zeros((5, 16, 32), dtype=np.uint8)
            press = rest.copy()
            rest[:, 4, 10] = 7
            press[:, 3, 20] = 3
            a, b = Path(directory)/'rest.npz', Path(directory)/'press.npz'
            np.savez(a, frames=rest, mux_offset=0)
            np.savez(b, frames=press, mux_offset=0)
            output = io.StringIO()
            with redirect_stdout(output):
                compare(a, b)
            self.assertIn('largest_median_change=-7.00 at row=4 mux=4 col=10', output.getvalue())
            np.savez(b, frames=press[:, :12], mux_offset=4)
            with self.assertRaises(ValueError):
                compare(a, b)


if __name__ == '__main__':
    unittest.main()
