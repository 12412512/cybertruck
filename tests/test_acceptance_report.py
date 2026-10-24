"""验收历史记录永远不能提升为实时状态或实车动作许可。"""
import json
from pathlib import Path
import sys
import tempfile
import unittest

from fastapi.testclient import TestClient

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pi'))
from robot.device.acceptance import build_report, read_report
from robot.web.app import create_app


class AcceptanceTests(unittest.TestCase):
    def setUp(self):
        self.folder=tempfile.TemporaryDirectory(); self.path=Path(self.folder.name)/'source.json'
        self.execution=dict(fault=0,can_errors=0,action_id=0,action_state=0,config_version=0,stop_confirmed=1,board_ms=500)
        self.data=dict(summary=dict(passed=True,firmware='GC-BASE-0.3.11',boot_id='boot',requests={'1':1,'35':5},
                                    checks={},wheels=[],can_rx_delta=100,uart_diag=[],odom_fault_record={}),
                       states=[dict(execution=self.execution)],feedback=[])

    def tearDown(self):
        self.folder.cleanup()

    def build(self):
        self.path.write_text(json.dumps(self.data),encoding='utf-8')
        return build_report(self.path,1792843823.)

    def test_pass_is_historical_not_live_or_permission(self):
        report=self.build()
        self.assertTrue(report['passed'])
        for key in ('live','connected','authorizes_motion'): self.assertFalse(report[key])

    def test_fault_overrides_claimed_pass_and_preserves_transition(self):
        self.data['states'].append(dict(execution=self.execution|dict(fault=2048,can_errors=1,board_ms=24464)))
        report=self.build()
        self.assertFalse(report['passed'])
        self.assertEqual(len(report['transition']),2)
        self.assertEqual(report['transition'][-1]['fault'],2048)

    def test_nonreadonly_or_nonzero_action_cannot_pass(self):
        self.data['summary']['requests']['33']=1
        self.assertFalse(self.build()['passed'])
        self.data['summary']['requests'].pop('33')
        self.execution['action_id']=1
        self.assertFalse(self.build()['passed'])

    def test_can_capture_remains_historical_with_no_permission(self):
        self.data['summary']['requests']['38'] = 65
        self.data['summary']['can_fault_record'] = dict(complete=True, first=dict(hal_error=4096), history=[])
        self.execution.update(fault=2048, can_errors=1)
        report = self.build()
        self.assertTrue(report['readonly_commands'])
        self.assertFalse(report['passed'])
        self.assertTrue(report['can_fault_record']['complete'])
        self.assertFalse(report['authorizes_motion'])

    def test_report_reader_forces_no_connection_or_permission(self):
        report=self.build()|dict(connected=True,authorizes_motion=True)
        self.path.write_text(json.dumps(report),encoding='utf-8')
        view=read_report(self.path)
        self.assertTrue(view['available'])
        self.assertFalse(view['connected']); self.assertFalse(view['authorizes_motion'])

    def test_missing_or_live_report_rejected(self):
        self.assertFalse(read_report(self.path)['available'])
        report=self.build()|dict(live=True)
        self.path.write_text(json.dumps(report),encoding='utf-8')
        self.assertFalse(read_report(self.path)['available'])

    def test_get_api_only_and_does_not_contact_core(self):
        self.path.write_text(json.dumps(self.build()),encoding='utf-8')
        class NoCore:
            def request(self,*args): raise AssertionError('验收归档不得访问执行核心')
        with TestClient(create_app(self.folder.name,NoCore(),self.path)) as client:
            response=client.get('/api/workbench/acceptance')
            self.assertEqual(response.status_code,200)
            self.assertFalse(response.json()['live'])
            self.assertEqual(client.post('/api/workbench/acceptance',json={}).status_code,405)


if __name__=='__main__': unittest.main()
