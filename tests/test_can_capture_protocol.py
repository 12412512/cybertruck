"""真实C分页与Python解析交叉验证；错误页不能变成完整现场或动作许可。"""
import os
import struct
import subprocess
import unittest
from test_execution import ROOT
import test_can_query_capture as fixture
from robot.device.can_capture import CaptureReader, decode_page, CAN_CAPTURE
from robot.device.execution import MotionLink
from robot.device import protocol as p


class CanCaptureProtocolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture.CanQueryCaptureTests.setUpClass()
        result = subprocess.run([str(fixture.CanQueryCaptureTests.binary), '0'],
                                env=dict(os.environ, CAN_DUMP_PAGES='1'), capture_output=True, check=True)
        cls.pages = [bytes.fromhex(line.split()[2]) for line in result.stdout.decode().splitlines()
                     if line.startswith('PAGE ')]

    def test_real_c_pages_decode_all_64_events(self):
        reader = CaptureReader()
        for page in self.pages:
            reader.feed(page, 123, 456)
        result = reader.result
        self.assertTrue(result['complete'])
        self.assertEqual(len(result['history']), 64)
        self.assertEqual(result['first']['tsr'], 0x1C000009)
        self.assertEqual(result['history'][-1]['kind_name'], 'ERROR')
        self.assertEqual(result['boot_id'], '000000000000007b')

    def test_partial_capture_is_not_complete(self):
        reader = CaptureReader()
        for page in self.pages[:-1]: reader.feed(page, 123, 456)
        self.assertIsNone(reader.result)

    def test_invalid_header_length_and_order_rejected(self):
        for page in (b'', self.pages[0][:-1], self.pages[1]):
            with self.assertRaises(ValueError): decode_page(page, 0)
        for offset, value in ((0, 2), (12, 65), (16, 63), (24, 41), (28, 1)):
            page = bytearray(self.pages[0]); struct.pack_into('<I', page, offset, value)
            with self.assertRaises(ValueError): decode_page(page, 0)

    def test_no_fault_header_keeps_reader_empty(self):
        reader = CaptureReader()
        reader.feed(struct.pack('<8I', 1, 0, 7, 0, 64, 0, 40, 0), 123, 456)
        self.assertEqual(reader.next_page, 0)
        self.assertIsNone(reader.result)
        self.assertEqual(reader.header, dict(version=1, valid=False, generation=7, count=0))

    def test_mixed_generation_or_identity_rejected(self):
        for boot, session in ((124, 456), (123, 457)):
            reader = CaptureReader(); reader.feed(self.pages[0], 123, 456)
            with self.assertRaises(ValueError): reader.feed(self.pages[1], boot, session)
        reader = CaptureReader(); reader.feed(self.pages[0], 123, 456)
        page = bytearray(self.pages[1]); struct.pack_into('<I', page, 8, 999)
        with self.assertRaises(ValueError): reader.feed(page, 123, 456)

    def test_event_sequence_corruption_rejected(self):
        page = bytearray(self.pages[1]); struct.pack_into('<I', page, 32, 999)
        with self.assertRaises(ValueError): decode_page(page, 1)

    def link(self):
        link = MotionLink(); link.boot = 123; link.session = 456; link.bound = True
        link.firmware = 'GC-BASE-0.3.12'; link.execution = dict(mode=6, can_errors=1)
        link.last_execution = link.last_fault_query = link.last_sample = 10
        return link

    def test_new_version_only_reads_in_static_modes(self):
        link = self.link()
        self.assertEqual(link.extra_request(10), (CAN_CAPTURE, b'\0'))
        link.execution['mode'] = 4
        self.assertIsNone(link.extra_request(10))
        link.execution['mode'] = 6; link.firmware = 'GC-BASE-0.3.11'
        self.assertIsNone(link.extra_request(10))

    def test_paging_reply_and_snapshot_preserve_identity(self):
        link = self.link()
        for index, page in enumerate(self.pages):
            request = p.Packet(CAN_CAPTURE, 123, 456, index, 250, bytes([index]))
            packet = p.Packet(CAN_CAPTURE | 0x80, 123, 456, index, 250, bytes([p.OK]) + page)
            self.assertTrue(link.custom_reply(packet, request, 10))
        self.assertTrue(link.can_capture.result['complete'])
        self.assertIsNone(link.extra_request(10))
        link.reset('模拟重连')
        self.assertIsNone(link.can_capture.result)
        self.assertEqual(link.can_capture.next_page, 0)

    def test_bad_identity_is_diagnostic_error_not_permission(self):
        link = self.link()
        request = p.Packet(CAN_CAPTURE, 123, 456, 1, 250, b'\0')
        packet = p.Packet(CAN_CAPTURE | 0x80, 124, 456, 1, 250, b'\0' + self.pages[0])
        link.custom_reply(packet, request, 10)
        self.assertTrue(link.can_capture_error)
        self.assertIsNone(link.can_capture.result)
        self.assertIsNone(link.extra_request(10))
        with self.assertRaises(ValueError): link.submit(CAN_CAPTURE)

    def test_stop_preempts_pending_capture_page(self):
        link = self.link(); link.last_heart = 10
        packet = p.Packet(CAN_CAPTURE, 123, 456, 1, 250, b'\0')
        link.pending = (packet, 10); link.stop()
        result = p.Decoder().feed(link.step(10))
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].kind, p.STOP)


if __name__ == '__main__': unittest.main()
