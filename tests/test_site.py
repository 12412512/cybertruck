"""人工测量配准的合成正反例；不将夹具写为实车测量。"""

from copy import deepcopy
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'pi'))
from robot.nav.site import read_site, readiness, initial_map_pose, bind, valid, require_north
from robot.nav.frames import Pose, compose


def fixture():
    site = deepcopy(read_site())
    site['field'].update(width_m=2.4, height_m=2.4, layout_matches_reference=True)
    site['body'].update(front_m=.145, rear_m=.145, left_m=.135, right_m=.135, complete=True)
    site['start'].update(from_left_m=.15, from_top_m=.15, position_error_m=.002,
                         heading_error_rad=math.radians(.5))
    site['start_zone'].update(width_m=.3, height_m=.3)
    site['target'].update(node='north', map_x_m=1.2, map_y_m=2.12, floor_mark_measured=True)
    site['corridor'].update(state='OPEN', minimum_width_m=.4)
    return site


class SiteTests(unittest.TestCase):
    def test_explicit_disabled_guard_does_not_fake_registration(self):
        site = fixture()
        site['field_execution_guard_enabled'] = False
        site['start']['heading_error_rad'] = None
        with patch('robot.nav.site.read_site', return_value=site):
            self.assertIsNone(require_north())
        self.assertFalse(readiness(site)['registration_valid'])

    def test_incomplete_record_does_not_allow_registration(self):
        site = fixture()
        site['body']['complete'] = False
        site['body']['front_m'] = None
        state = readiness(site)
        self.assertEqual(state['status'], 'WAITING_MEASUREMENTS')
        self.assertFalse(state['executable'])
        self.assertFalse(state['registration_valid'])

    def test_unknown_start_or_body_never_becomes_nominal_measurement(self):
        for group, key in (('body', 'front_m'), ('start', 'from_left_m'), ('target', 'map_y_m')):
            site = fixture()
            site[group][key] = None
            with self.assertRaises(ValueError):
                initial_map_pose(site)

    def test_top_distance_is_not_map_y(self):
        self.assertEqual(initial_map_pose(fixture()), Pose(.15, 2.25, 0))

    def test_binding_handles_nonzero_odom_without_zeroing(self):
        site = fixture()
        odom = Pose(.7, -.3, .4)
        binding = bind(site, odom, 123, 2, 10)
        restored = compose(Pose(*binding['map_to_odom']), odom)
        for a, b in zip(restored.values(), initial_map_pose(site).values()):
            self.assertAlmostEqual(a, b, places=10)
        self.assertTrue(valid(binding, 123, 2, site))
        self.assertFalse(valid(binding, 124, 2, site))
        self.assertFalse(valid(binding, 123, 3, site))
        site['start']['from_left_m'] += .01
        self.assertFalse(valid(binding, 123, 2, site))

    def test_stale_odom_or_nan_measurement_rejected(self):
        for age in (-1, 250, float('nan')):
            with self.assertRaises(ValueError):
                bind(fixture(), Pose(0, 0, 0), 123, 1, age)
        site = fixture()
        site['body']['left_m'] = float('nan')
        self.assertEqual(readiness(site)['status'], 'INVALID_DATA')

    def test_complete_data_is_not_motion_authorization(self):
        state = readiness(fixture())
        self.assertEqual(state['status'], 'DATA_COMPLETE')
        self.assertFalse(state['executable'])

    def test_edge_touching_start_is_not_robustly_clear(self):
        site = fixture()
        site['start'].update(from_left_m=.145, from_top_m=.135)
        state = readiness(site)
        self.assertEqual(state['status'], 'PLACEMENT_UNSAFE')
        self.assertEqual(state['placement']['field_clearance_m']['rear'], 0)
        self.assertEqual(state['placement']['field_clearance_m']['left'], 0)
        with self.assertRaises(ValueError):
            bind(site, Pose(0, 0, 0), 123, 1, 0)

    def test_boundary_risk_visible_before_remaining_data_complete(self):
        site = fixture()
        site['start'].update(from_left_m=.145, from_top_m=.135, position_error_m=None)
        site['corridor']['state'] = 'UNKNOWN'
        state = readiness(site)
        self.assertEqual(state['status'], 'PLACEMENT_UNSAFE')
        self.assertIsNone(state['placement']['placement_inflation_m'])
        self.assertIn('corridor.confirmed_open', state['missing'])

    def test_planned_target_is_not_required_to_be_a_localization_landmark(self):
        site = fixture()
        site['target']['floor_mark_measured'] = False
        self.assertEqual(readiness(site)['status'], 'DATA_COMPLETE')


if __name__ == '__main__':
    unittest.main()
