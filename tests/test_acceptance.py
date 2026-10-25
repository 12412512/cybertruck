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


class ModernAcceptanceTests(unittest.TestCase):
    def setUp(self):
        AcceptanceTests.setUp(self)
        self.requests += [37,38,58,60]
        self.link.mech_state=dict(pwm_mask=0,axis_state=0,servo_state=0,pose_state=0,motion_uncertain=0)
        for row in self.states:
            row.update(firmware='GC-BASE-0.3.21',session_id='one',physical_start=dict(ticket=0,fired=0,beep=0,
                select_zone=1,select_valid=1),can_capture_status=dict(valid=0),can_capture_error=None,
                odom_fault_record=dict(valid=0),board=dict(flags=0))
        self.states[-1]['physical_start']['select_zone']=2
        self.states[-1]['board']['flags']=8

    def result(self):
        return summarize(self.link,self.states,self.requests,'GC-BASE-0.3.21')

    def test_modern_readonly_inputs_and_pwm_pass(self):
        from accept_board import analyze
        summary=self.result()
        self.assertTrue(summary['passed'])
        report=analyze(dict(summary=summary,states=self.states))
        self.assertTrue(report['passed'])
        self.assertFalse(report['full_hardware_accepted'])
        self.assertFalse(report['authorizes_motion'])

    def test_prepare_or_attach_command_is_never_readonly(self):
        for kind in (32,33,34,48,49,50,51,53,55,59):
            self.requests.append(kind)
            self.assertFalse(self.result()['passed'])
            self.requests.pop()

    def test_can_candidate_readonly_checks_still_require_clean_fault_record(self):
        for row in self.states:
            row['firmware'] = 'GC-BASE-0.3.22'
        self.assertTrue(summarize(self.link,self.states,self.requests,'GC-BASE-0.3.22')['passed'])
        self.states[-1]['execution']['can_errors'] = 1
        self.assertFalse(summarize(self.link,self.states,self.requests,'GC-BASE-0.3.22')['passed'])

    def test_identity_ticket_pwm_and_missing_first_error_rejected(self):
        self.states[-1]['session_id']='other'
        self.assertFalse(self.result()['passed'])
        self.states[-1]['session_id']='one'
        self.states[-1]['physical_start']['ticket']=1
        self.assertFalse(self.result()['passed'])
        self.states[-1]['physical_start']['ticket']=0
        self.link.mech_state['pwm_mask']=1
        self.assertFalse(self.result()['passed'])
        self.link.mech_state['pwm_mask']=0
        self.states[-1]['can_capture_status']=None
        self.assertFalse(self.result()['passed'])

    def test_default_board_tool_does_not_collect(self):
        import contextlib,io
        from accept_board import main
        output=io.StringIO()
        with patch('sys.argv',['accept_board','--port','not-a-device']),patch('accept_board.subprocess.run') as run:
            with contextlib.redirect_stdout(output):main()
        import json
        self.assertFalse(json.loads(output.getvalue())['device_access'])
        run.assert_not_called()

    def test_board_analysis_does_not_trust_old_pass_after_fault_or_action(self):
        from accept_board import analyze
        data = dict(summary=self.result(), states=self.states)
        self.states[-1]['execution']['fault'] = 2048
        self.assertFalse(analyze(data)['passed'])
        self.states[-1]['execution']['fault'] = 0
        data['summary']['requests']['59'] = 1
        self.assertFalse(analyze(data)['passed'])


if __name__ == '__main__':
    unittest.main()
