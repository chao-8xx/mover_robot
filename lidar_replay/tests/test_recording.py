import json
import struct
import unittest
from pathlib import Path

from recording import RecordingError, decode_d6, normalize_recording, read_text_recording


def packet(start, angles, distances, intensities=None):
    strengths = intensities or [0] * len(distances)
    p = bytearray([0xAA, 0x55, int(start), len(distances)])
    for angle in (angles[0], angles[-1]):
        p.extend(struct.pack('<H', (int(angle * 64) << 1) | 1))
    p.extend([0, 0])
    for d, strength in zip(distances, strengths):
        p.extend([(strength & 63) << 2, ((d & 63) << 2) | (strength >> 6), d >> 6])
    xor = 0
    for i in (0, 2, 4, 6):
        xor ^= p[i] | p[i + 1] << 8
    for i in range(10, len(p), 3):
        xor ^= p[i] | p[i + 1] << 8
        xor ^= p[i + 2]
    p[8:10] = struct.pack('<H', xor)
    return bytes(p)


class RecordingTests(unittest.TestCase):
    def test_dense_preserves_validity_and_missing_pose(self):
        frame = {'distance': [100] * 360, 'valid': [0] * 360}
        result = normalize_recording({'frames': [frame]})['frames'][0]
        self.assertFalse(result['pose']['valid'])
        self.assertEqual(sum(result['valid']), 0)

    def test_sparse_nearest_same_angle(self):
        result = normalize_recording({'points': [{'angle': 3.9, 'distance': 500},
                                                 {'angle': 3.1, 'distance': 400, 'intensity': 250}]})
        frame = result['frames'][0]
        self.assertEqual((frame['distance'][3], frame['intensity'][3]), (400, 250))

    def test_reject_zero_valid_distance_nan_duplicate_scan_and_bad_angle(self):
        bad = [ {'points': [{'angle': 0, 'distance': 0, 'valid': True}]},
                {'points': [{'angle': -1, 'distance': 100}]},
                {'points': [{'angle': 0, 'distance': float('nan')}]},
                {'frames': [{'scan_num': 1, 'points': []}, {'scan_num': 1, 'points': []}]}]
        for data in bad:
            with self.subTest(data=data), self.assertRaises(RecordingError):
                normalize_recording(data)

    def test_csv_pose_must_be_consistent(self):
        text = 'scan_num,angle,distance,pose_x,pose_y,pose_yaw,pose_valid\n1,270,800,1200,800,0,1\n1,271,800,1200,800,0,1\n'
        rec = read_text_recording(text, 'record.csv')
        self.assertTrue(rec['frames'][0]['pose']['valid'])
        with self.assertRaises(RecordingError):
            read_text_recording(text + '1,272,800,1201,800,0,1\n', 'record.csv')

    def test_jsonl_and_summary_error(self):
        text = '\n'.join(json.dumps({'scan_num': i, 'points': []}) for i in (2, 4))
        self.assertEqual(len(read_text_recording(text, 'x.jsonl')['frames']), 2)
        with self.assertRaisesRegex(RecordingError, '统计日志'):
            read_text_recording('SCAN n=1 dt=100ms raw=400 valid=200', 'uart.txt')

    def test_synthetic_jsonl_label_is_retained(self):
        text = json.dumps({'scan_num': 1, 'points': [], 'metadata': {'synthetic': True}})
        self.assertTrue(read_text_recording(text, 'example.jsonl')['synthetic'])

    def test_problem_export_restores_settings_and_selected_frame(self):
        data = {'frames': [{'scan_num': 1, 'points': []}, {'scan_num': 2, 'points': []}],
                'params': {'min_points': 1}, 'car_width_mm': 600, 'safety_mm': 80,
                'problem_frame_index': 1, 'compare': True}
        restored = read_text_recording(json.dumps(data), 'problem.json')
        for field in ('params', 'car_width_mm', 'safety_mm', 'problem_frame_index', 'compare'):
            self.assertEqual(restored[field], data[field])

    def test_delivered_sample_formats_preserve_published_bins(self):
        directory = Path(__file__).resolve().parents[1] / 'samples'
        records = [read_text_recording((directory / name).read_text(encoding='utf-8'), name)
                   for name in ('synthetic_demo.json', 'synthetic_demo.jsonl', 'synthetic_demo.csv')]
        for rec in records:
            self.assertTrue(rec['synthetic'])
            self.assertEqual(len(rec['frames']), 40)
        for a, b, c in zip(*(rec['frames'] for rec in records)):
            for field in ('distance', 'valid', 'intensity', 'scan_num', 'pose'):
                self.assertEqual(a[field], b[field])
                self.assertEqual(a[field], c[field])

    def test_d6_nearest_intensity_zero_and_boundaries(self):
        raw = packet(False, [90], [123]) + packet(True, [0], [0])
        raw += packet(False, [358, 2], [500, 400, 300], [1, 200, 250])
        raw += packet(False, [0], [200], [99])
        raw += packet(True, [0], [123])
        rec = decode_d6(raw)
        self.assertEqual(len(rec['frames']), 1)
        frame = rec['frames'][0]
        self.assertEqual(frame['raw_count'], 5)
        self.assertEqual(frame['distance'][0], 200)
        self.assertEqual(frame['intensity'][0], 99)
        self.assertEqual(frame['distance'][358], 500)
        self.assertEqual(frame['distance'][2], 300)
        self.assertFalse(frame['pose']['valid'])
        self.assertFalse(rec['metadata']['timestamp_known'])

    def test_bad_checksum_not_accepted(self):
        bad = bytearray(packet(False, [30], [100]))
        bad[-1] ^= 1
        rec = decode_d6(packet(True, [0], [50]) + bytes(bad) + packet(True, [0], [50]) + packet(True, [0], [50]))
        self.assertEqual(rec['metadata']['decoder']['xor_error'], 1)
        self.assertEqual(rec['frames'][0]['valid'][30], 0)

    def test_truncated_recording_no_false_full_scan(self):
        with self.assertRaises(RecordingError):
            decode_d6(packet(True, [0], [100]) + packet(False, [1], [200])[:-1])


if __name__ == '__main__':
    unittest.main()
