"""人工固定雷达的单次隔离校准；不降低普通扫描和底盘门槛。"""
import unittest
from test_lidar_calibrate import CalibrationTests
from test_lidar_scan import board
from robot.lidar_core import board_still, calibration_still


class IsolatedTests(CalibrationTests):
    def test_locked_calibration_once(self):
        core, node, now, _ = self.prepare(write=True, fault=True)
        core.calibration_can_locked = True
        request = self.request(core).model_copy(update={'allow_can_latched': True})
        core.commit_calibration(request)
        self.assertFalse(core.calibration_write_enabled)
        for _ in range(1200):
            state = board(); state['base'].update(mode=6, fault=0x800)
            core.board(state, now[0]); core.tick(node); now[0] += .01
            if core.job is None: break
        self.assertEqual(core.last_calibration['status'], 'COMPLETE')
        self.assertEqual(node.commands.count(9), 1)
        self.assertEqual(core.witness['base']['fault'], 0x800)
        self.assertTrue(core.last_calibration['stop_confirmed'])
        self.assertFalse(board_still(state, now[0], now[0]))

    def test_isolation_rejects_unsafe_state(self):
        state = board(); state['base'].update(mode=6, fault=0x800)
        self.assertTrue(calibration_still(state, 1., 1., True))
        self.assertFalse(calibration_still(state, 1., 1., False))
        for section, key, value in [('base','fault',1), ('base','stopped',False),
                                    ('mechanism','pwm_mask',1), ('mechanism','axis_state',1),
                                    ('mechanism','fault',14)]:
            changed = {**state, section:{**state[section],key:value}}
            self.assertFalse(calibration_still(changed, 1., 1., True))
        self.assertFalse(calibration_still(state, 1., 1.25, True))

    def test_isolation_requires_server_opt_in(self):
        core, node, now, _ = self.prepare(write=True, fault=True)
        request = self.request(core).model_copy(update={'allow_can_latched': True})
        with self.assertRaises(ValueError): core.commit_calibration(request)
        self.assertNotIn(9, node.commands)


if __name__ == '__main__':
    unittest.main()
