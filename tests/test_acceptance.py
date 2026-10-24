"""只读实车验收判据的负例；仅合成数据，不访问设备。"""

from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from accept_base import summarize
from robot.main import open_transport


class AcceptanceTests(unittest.TestCase):
    def setUp(self):
        records = [dict(kind=kind, address=address, values=[3 if kind == 3 else 0, 0, 0])
                   for address in range(1, 5) for kind in (1, 2, 3) for _ in range(3)]
        records += [dict(kind=kind, address=0, values=[0, 0, 0]) for kind in (4, 5) for _ in range(5)]
        self.link = SimpleNamespace(records=records, decoder=SimpleNamespace(bad=0),
                                    uart_diag=[dict(overflows=0, errors=0, restart_errors=0)] * 2)
        state = dict(firmware='GC-BASE-0.3.6', boot_id='test', execution=dict(
            mode=0, fault=0, action_id=0, action_state=0, config_version=0,
            stop_confirmed=1, can_errors=0, can_rx_sequence=10))
        self.states = [state, deepcopy(state)]
        self.states[-1]['execution']['can_rx_sequence'] = 20
        self.requests = [1, 2, 3, 4, 5, 16, 35, 36]

    def result(self):
        return summarize(self.link, self.states, self.requests)

    def test_healthy_readonly_window_passes(self):
        self.assertTrue(self.result()['passed'])

    def test_latched_fault_never_passes(self):
        self.states[-1]['execution'].update(mode=6, fault=258)
        self.assertFalse(self.result()['passed'])

    def test_old_firmware_never_passes(self):
        self.states[-1]['firmware'] = 'GC-BASE-0.3.0'
        self.assertFalse(self.result()['passed'])

    def test_motion_or_configuration_request_never_passes(self):
        for kind in (6, 32, 33, 34):
            self.requests.append(kind)
            self.assertFalse(self.result()['passed'])
            self.requests.pop()

    def test_uart_error_never_passes(self):
        self.link.uart_diag = [dict(overflows=0, errors=1, restart_errors=0)]
        self.assertFalse(self.result()['passed'])

    def test_nonzero_wheel_and_missing_channel_never_pass(self):
        self.link.records[0]['values'][0] = 1
        self.assertFalse(self.result()['passed'])
        self.link.records = [r for r in self.link.records if (r['kind'], r['address']) != (3, 4)]
        self.assertFalse(self.result()['passed'])

    def test_frozen_can_sequence_never_passes(self):
        self.states[-1]['execution']['can_rx_sequence'] = 10
        self.assertFalse(self.result()['passed'])


class StartupTests(unittest.TestCase):
    def test_quiet_window_precedes_device_handoff(self):
        device = Mock()
        events = []
        def opened():
            self.assertFalse(device.dtr)
            self.assertFalse(device.rts)
            events.append('opened')
        device.open.side_effect = opened
        def settled(seconds):
            self.assertEqual(seconds, .5)
            self.assertEqual(events, ['opened'])
            device.write.assert_not_called()
            events.append('settled')
        with patch.dict(sys.modules, serial=SimpleNamespace(Serial=Mock(return_value=device))), \
                patch('robot.main.time.sleep', side_effect=settled):
            self.assertIs(open_transport('serial', 'test-port'), device)
        self.assertEqual(events, ['opened', 'settled'])

    def test_interrupted_startup_closes_port(self):
        device = Mock()
        with patch.dict(sys.modules, serial=SimpleNamespace(Serial=Mock(return_value=device))), \
                patch('robot.main.time.sleep', side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                open_transport('serial', 'test-port')
        device.close.assert_called_once()
        device.write.assert_not_called()


if __name__ == '__main__':
    unittest.main()
