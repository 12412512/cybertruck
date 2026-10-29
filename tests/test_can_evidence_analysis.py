"""寄存器解释与条件性冲突模型，不把仿真当作物理根因证明。"""
import importlib.util
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('can_capture_analysis',ROOT/'tools/analyze_can_capture.py')
analysis=importlib.util.module_from_spec(spec); spec.loader.exec_module(analysis)


class CanEvidenceAnalysisTests(unittest.TestCase):
    def test_ram_bit_timing(self):
        value=analysis.bit_timing(0x1B0003,30000000)
        self.assertEqual(value['nominal_bitrate'],500000)
        self.assertAlmostEqual(value['sample_point'],13/15)
        self.assertEqual(value['sjw'],1)

    def test_same_id_data_frames_cannot_arbitrate_on_dlc(self):
        value=analysis.simultaneous_data_frames(0x300,b'\x3a\x6b',0x300,b'\x3a\x03\x6b')
        self.assertFalse(value['arbitration_can_distinguish'])
        self.assertEqual(value['divergence'],'dlc')
        self.assertFalse(value['physical_overlap_proven'])

    def test_different_ids_and_identical_frames(self):
        self.assertTrue(analysis.simultaneous_data_frames(0x300,b'a',0x200,b'bb')['arbitration_can_distinguish'])
        self.assertEqual(analysis.simultaneous_data_frames(0x300,b'a',0x300,b'a')['divergence'],'none')

    def test_conditional_delayed_reply_barrier(self):
        value=analysis.reply_barrier_model(10.)
        self.assertEqual(value['periodic_same_sof_ms'],[10.])
        self.assertEqual(value['paced_same_sof_ms'],[])
        self.assertFalse(value['physical_proof'])

    def test_bus_busy_overlap_is_not_assumed_simultaneous(self):
        self.assertEqual(analysis.reply_barrier_model(9.9)['periodic_same_sof_ms'],[])

    def test_reply_timeout_stops_without_auto_retry(self):
        value=analysis.reply_barrier_model(101.)
        self.assertEqual(value['paced_status'],'reply_timeout_stop')
        self.assertEqual(value['paced_requests_ms'],[0.])


if __name__=='__main__': unittest.main()
