"""验证真实安装方向的平面符号；不连接设备。"""
import json
import math
from pathlib import Path
import unittest

from robot.lidar.geometry import local_to_map, radar_pose


class GeometryTests(unittest.TestCase):
    def setUp(self):
        root = Path(__file__).resolve().parents[1]
        self.installation = json.loads((root/'config/lidar_installation.json').read_text(encoding='utf-8'))

    def test_zone1_and_rotated_body_move_installation_offset_once(self):
        for body, expected in (([.15, 2.25, 0], [.15, 2.105, -math.pi/2]),
                ([2.25, 2.25, math.pi], [2.25, 2.395, math.pi/2])):
            for value, target in zip(radar_pose(self.installation, body), expected):
                self.assertAlmostEqual(value, target)

    def test_sensor_forward_is_body_right_and_sensor_left_is_body_forward(self):
        for point, expected in (([.3, 0], [0, -.445]), ([0, .3], [.3, -.145])):
            result = local_to_map(dict(frame_id='lidar_local', position_m=point),
                self.installation, [0, 0, 0])
            for value, target in zip(result['position_m'], expected):
                self.assertAlmostEqual(value, target)
            self.assertFalse(result['authorizes_motion'])
            self.assertFalse(result['real_map_registered'])

    def test_vendor_field_unknown_heading_and_nonfinite_coordinates_rejected(self):
        with self.assertRaises(ValueError):
            local_to_map(dict(frame_id='cyqm_field', position_m=[.8, 1.5]), self.installation, [0, 0, 0])
        for body in ([0, 0, math.nan], [0, 0], [False, 0, 0]):
            with self.assertRaises(ValueError):
                radar_pose(self.installation, body)
        incomplete = dict(self.installation, mount_yaw_rad=None)
        with self.assertRaises(ValueError):
            radar_pose(incomplete, [0, 0, 0])


if __name__ == '__main__':
    unittest.main()
