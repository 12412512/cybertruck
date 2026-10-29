"""厂家校准字段及新保存记录检查；纯离线，不发送09。"""
import copy
import struct
import unittest

from test_lidar_health import saved_data
from robot.lidar.protocol import persisted, frame
from robot.lidar.vendor_calibration import decode_result, verify_saved


def values(zone=1):
    old = persisted(saved_data())
    payload = struct.pack('<BhhHhhhBB', zone, 160, 2100, 180*64, 10, -150, -4*64, 4, 5)
    result = decode_result(payload)
    current = copy.deepcopy(old)
    current.update(storage_version=11, source_zone=zone,
        record_sequence=(old['record_sequence']+1) & 0xffffffff)
    old['storage_version'] = 11
    current[f'zone{zone}_pose'] = {key:result[key] for key in ('zone', 'x_mm', 'y_mm', 'heading_deg')}
    return old, current, result


class VendorCalibrationTests(unittest.TestCase):
    def test_default_encoder_does_not_open_calibration_or_pose_writes(self):
        for command in (0x06, 0x09, 0x11):
            with self.assertRaises(ValueError):
                frame(1, command, 0, b'\1')

    def test_sdk_fields_and_both_zone_save_consistency(self):
        for zone in (1, 2):
            old, current, result = values(zone)
            self.assertEqual(result['heading_trim_deg'], -4)
            self.assertEqual(result['y_offset_mm'], -150)
            verified = verify_saved(old, current, result, zone)
            self.assertTrue(verified['saved_consistent'])
            self.assertFalse(verified['real_map_registered'])
            self.assertFalse(verified['authorizes_motion'])

    def test_bad_size_zone_heading_rejected(self):
        for payload in (b'', bytes(14), struct.pack('<BhhHhhhBB', 0, 0, 0, 0, 0, 0, 0, 0, 0),
                struct.pack('<BhhHhhhBB', 1, 0, 0, 23040, 0, 0, 0, 0, 0)):
            with self.assertRaises(ValueError):
                decode_result(payload)

    def test_cached_or_backward_record_wrong_zone_version_pose_rejected(self):
        for case in ('cached', 'backward', 'zone', 'version', 'pose', 'other_zone'):
            old, current, result = values()
            if case == 'cached': current['record_sequence'] = old['record_sequence']
            if case == 'backward': current['record_sequence'] = (old['record_sequence']-1) & 0xffffffff
            if case == 'zone': current['source_zone'] = 2
            if case == 'version': current['storage_version'] = 12
            if case == 'pose': current['zone1_pose']['x_mm'] += 1
            if case == 'other_zone': current['zone2_pose']['x_mm'] += 1
            with self.subTest(case=case), self.assertRaises(ValueError):
                verify_saved(old, current, result, 1)

    def test_sequence_wrap_is_new_but_mismatched_result_is_not(self):
        old, current, result = values()
        old['record_sequence'] = 0xffffffff; current['record_sequence'] = 0
        self.assertTrue(verify_saved(old, current, result, 1)['saved_consistent'])
        result['zone'] = 2
        with self.assertRaises(ValueError):
            verify_saved(old, current, result, 1)


if __name__ == '__main__':
    unittest.main()
