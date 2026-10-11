"""Replay D6 byte streams through unchanged firmware lidar_process on host C.

The DMA producer is mocked; the parser, checksum, scan collector and publication
are the real current BSP/Src/lidar.c.  This verifies bytes, not DMA timing on board.
"""
import json
import os
from pathlib import Path
import random
import struct
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from recording import RecordingError, decode_d6
from replay_engine import firmware_dir
try:
    from .test_c_parity import find_host_cc
except ImportError:
    from test_c_parity import find_host_cc


HEADFILE = r'''
#ifndef REPLAY_D6_HEADFILE_H
#define REPLAY_D6_HEADFILE_H
#include <stdint.h>
#include <string.h>
#include <stdio.h>
typedef enum {HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2} HAL_StatusTypeDef;
typedef struct {uint16_t remaining; uint8_t pending_tc;} DMA_HandleTypeDef;
typedef struct {void *Instance; DMA_HandleTypeDef *hdmarx; uint32_t RxState;} UART_HandleTypeDef;
#define __HAL_DMA_GET_COUNTER(hdma) ((hdma)->remaining)
#define __HAL_DMA_GET_TC_FLAG_INDEX(hdma) (1U)
#define __HAL_DMA_GET_FLAG(hdma, flag) ((hdma)->pending_tc)
#define __get_PRIMASK() (0U)
#define __disable_irq() ((void)0)
#define __set_PRIMASK(value) ((void)(value))
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
#include "lidar.h"
#endif
'''


HARNESS = r'''
#include "headfile.h"
static uint8_t *host_rx;
static uint16_t host_size, host_write;
static uint32_t host_tick;
uint32_t HAL_GetTick(void) {return host_tick++;}
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t size) {
    host_rx=buffer; host_size=size; host_write=0;
    uart->hdmarx->remaining=size; uart->hdmarx->pending_tc=0;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t size) {
    (void)uart; (void)buffer; (void)size; return HAL_OK;
}
static void print_scan(const lidar_scan_t *scan) {
    printf("{\"type\":\"frame\",\"scan_num\":%lu,\"start_time\":%lu,\"end_time\":%lu,"
           "\"raw_count\":%u,\"valid_count\":%u,\"points\":[",
           (unsigned long)scan->scan_num, (unsigned long)scan->start_time,
           (unsigned long)scan->end_time, scan->raw_count, scan->valid_count);
    for (unsigned a=0; a<360; ++a)
        printf("%s[%u,%u,%u]", a ? "," : "", scan->distance[a], scan->valid[a], scan->intensity[a]);
    puts("]}");
}
int main(void) {
    DMA_HandleTypeDef dma={0};
    UART_HandleTypeDef uart={0};
    unsigned byte;
    uint32_t last_scan=0;
    uart.hdmarx=&dma;
    lidar_init(&uart);
    while (scanf("%u", &byte)==1) {
        if (byte>255) return 2;
        host_rx[host_write++]=(uint8_t)byte;
        if (host_write==host_size) {
            host_write=0; dma.remaining=host_size;
        } else dma.remaining=host_size-host_write;
        lidar_process();
        const lidar_scan_t *scan=lidar_get_scan();
        if (scan && scan->scan_num!=last_scan) {
            print_scan(scan); last_scan=scan->scan_num;
        }
    }
    printf("{\"type\":\"stats\",\"xor_ok\":%lu,\"xor_error\":%lu,\"scan_drop\":%lu}\n",
           (unsigned long)lidar_get_xor_ok_cnt(), (unsigned long)lidar_get_xor_err_cnt(),
           (unsigned long)lidar_get_scan_drop_cnt());
    return 0;
}
'''


def checksum(packet):
    value = 0
    for offset in (0, 2, 4, 6):
        value ^= packet[offset] | (packet[offset + 1] << 8)
    for offset in range(10, len(packet), 3):
        value ^= packet[offset] | (packet[offset + 1] << 8)
        value ^= packet[offset + 2]
    return value


def packet(start=False, a0=0, a1=None, distances=(500,), intensities=None, fsa_raw=None, lsa_raw=None):
    a1 = a0 if a1 is None else a1
    fsa_raw = (int(a0 * 64) << 1) | 1 if fsa_raw is None else fsa_raw
    lsa_raw = (int(a1 * 64) << 1) | 1 if lsa_raw is None else lsa_raw
    strengths = intensities if intensities is not None else [0] * len(distances)
    result = bytearray([0xAA, 0x55, int(start), len(distances)])
    result.extend(struct.pack("<HHH", fsa_raw, lsa_raw, 0))
    for distance, strength in zip(distances, strengths):
        result.extend(((strength & 63) << 2, ((distance & 63) << 2) | (strength >> 6), distance >> 6))
    result[8:10] = struct.pack("<H", checksum(result))
    return bytes(result)


class D6CParityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = find_host_cc()
        if compiler is None:
            raise unittest.SkipTest("No runnable host compiler; D6 byte-stream differential checks not performed")
        cls.tempdir = tempfile.TemporaryDirectory(prefix="d6_c_parity_")
        cls.addClassCleanup(cls.tempdir.cleanup)
        directory = Path(cls.tempdir.name)
        project = firmware_dir(Path(__file__).resolve().parents[2])
        (directory / "headfile.h").write_text(HEADFILE, encoding="utf-8")
        (directory / "d6.c").write_text(HARNESS, encoding="utf-8")
        cls.executable = directory / ("d6.exe" if os.name == "nt" else "d6")
        cls.env = os.environ.copy()
        cls.env["PATH"] = str(Path(compiler).parent) + os.pathsep + cls.env.get("PATH", "")
        subprocess.run([compiler, "-std=c99", "-O0", "-ffp-contract=off", "-I", str(directory),
                        "-I", str(project / "BSP/Inc"), str(directory / "d6.c"),
                        str(project / "BSP/Src/lidar.c"), "-o", str(cls.executable)],
                       env=cls.env, capture_output=True, text=True, timeout=30, check=True)

    def compare(self, data):
        completed = subprocess.run([str(self.executable)], input=" ".join(str(byte) for byte in data),
                                   capture_output=True, text=True, env=self.env, check=True, timeout=30)
        results = [json.loads(line) for line in completed.stdout.splitlines()]
        c_frames, stats = results[:-1], results[-1]
        self.assertEqual(stats["type"], "stats")
        if not c_frames:
            with self.assertRaises(RecordingError):
                decode_d6(data)
            return None, stats
        decoded = decode_d6(data)
        frames = decoded["frames"]
        self.assertEqual(len(frames), len(c_frames))
        for index, (frame, c_frame) in enumerate(zip(frames, c_frames)):
            with self.subTest(frame=index):
                for key in ("scan_num", "start_time", "end_time", "raw_count"):
                    self.assertEqual(frame[key], c_frame[key], key)
                self.assertEqual(sum(frame["valid"]), c_frame["valid_count"])
                for angle, point in enumerate(c_frame["points"]):
                    self.assertEqual([frame[name][angle] for name in ("distance", "valid", "intensity")], point, f"{angle}°")
        for key in ("xor_ok", "xor_error"):
            self.assertEqual(decoded["metadata"]["decoder"][key], stats[key], key)
        return decoded, stats

    def test_zero_checksum_nearest_intensity_crosszero_and_rounding(self):
        zero = packet(a0=33, distances=(10901,))
        self.assertEqual(zero[8:10], b"\x00\x00")
        zero_start = packet(start=True, distances=(10965,))
        self.assertEqual(zero_start[8:10], b"\x00\x00")
        # Exercise leading noise and repeated-AA header resynchronization.
        raw = b"\x00\x77\xAA\xAA" + packet(a0=90, distances=(123,))
        raw += packet(start=True, distances=(0,)) + zero
        raw += packet(a0=358.5, a1=1.5, distances=(900, 800, 700, 600, 500, 400, 300), intensities=(1, 2, 3, 4, 5, 6, 7))
        raw += packet(a0=0.1, a1=0.9, distances=(200, 250, 200), intensities=(250, 2, 9))
        raw += zero_start + packet(start=True, distances=(50,))
        decoded, stats = self.compare(raw)
        self.assertEqual(decoded["frames"][0]["distance"][33], 10901)
        self.assertEqual(decoded["frames"][0]["intensity"][0], 250)
        self.assertEqual(stats["xor_error"], 0)

    def test_bad_packets_discard_work_scan_and_restart_at_next_valid_start(self):
        bad_xor = bytearray(packet(a0=25, distances=(300,)))
        bad_xor[-1] ^= 1
        errors = [bytes(bad_xor), packet(a0=25, fsa_raw=25 * 128, distances=(300,)),
                  packet(a0=400, distances=(300,)), packet(start=True, distances=()),
                  packet(start=True, distances=(300, 400)), b"\xAA\x55\x00\x1A"]
        for bad in errors:
            with self.subTest(bad=bad.hex()):
                raw = packet(start=True) + packet(a0=10, distances=(200,)) + bad
                raw += packet(a0=20, distances=(400,)) + packet(start=True, distances=(600,))
                raw += packet(a0=30, distances=(700,)) + packet(start=True, distances=(800,))
                decoded, stats = self.compare(raw)
                self.assertEqual(len(decoded["frames"]), 1)
                self.assertEqual(decoded["frames"][0]["start_time"], 1)
                self.assertEqual(decoded["frames"][0]["distance"][10], 0)
                self.assertEqual(decoded["frames"][0]["distance"][30], 700)
                self.assertEqual(stats["scan_drop"], 1)

    def test_zero_nonstart_packet_does_not_discard_and_trailing_truncation_does_not_publish(self):
        raw = packet(start=True) + packet(distances=()) + packet(a0=10, distances=(300,))
        raw += packet(start=True) + packet(a0=20, distances=(400,))[:-2]
        decoded, stats = self.compare(raw)
        self.assertEqual(len(decoded["frames"]), 1)
        self.assertEqual(decoded["frames"][0]["distance"][10], 300)
        self.assertEqual(stats["scan_drop"], 0)

    def test_bad_start_never_falsely_completes_preceding_scan(self):
        raw = packet(start=True) + packet(start=True, distances=(500, 600)) + packet(start=True)
        self.compare(raw)

    def test_seeded_random_packet_streams_across_dma_wraps(self):
        rng = random.Random(20261003)
        for case in range(25):
            raw = bytearray()
            for cycle in range(7):
                raw.extend(packet(start=True, a0=rng.randrange(360), distances=(rng.randrange(16384),)))
                for _ in range(rng.randrange(5, 18)):
                    count = rng.randrange(1, 26)
                    a0 = rng.randrange(360 * 64) / 64
                    a1 = rng.randrange(360 * 64) / 64
                    distances = [rng.randrange(16384) for _ in range(count)]
                    strengths = [rng.randrange(256) for _ in range(count)]
                    p = bytearray(packet(a0=a0, a1=a1, distances=distances, intensities=strengths))
                    if rng.random() < 0.04:
                        p[8] ^= 1
                    raw.extend(p)
            raw.extend(packet(start=True) + packet(start=True))
            with self.subTest(stream=case):
                self.compare(bytes(raw))

    def test_raw_count_uses_firmware_uint16_width(self):
        block = packet(a0=10, a1=20, distances=(100,) * 25)
        raw = packet(start=True) + block * 2622 + packet(start=True)
        decoded, _ = self.compare(raw)
        self.assertEqual(decoded["frames"][0]["raw_count"], (1 + 2622 * 25) & 0xFFFF)


if __name__ == "__main__":
    unittest.main()
