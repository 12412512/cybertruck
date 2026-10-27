"""连接管理只用假SSH、假设备状态；不连接树莓派或串口。"""

import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
from fastapi import FastAPI
from fastapi.testclient import TestClient

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'pi'))
from robot.connection_host import decide
from robot.connection_core import Connections, ConnectCommand, create_app, ForwardHandler
from robot.web.connections import router

PROFILE = dict(host='test-host', username='pi')


class FakeBackend:
    active = False
    password = ''

    def __init__(self, fail=None):
        self.calls = []
        self.fail = fail

    def connect(self, password):
        self.calls.append(('ssh',)); self.active = True; self.password = password

    def forward(self, component):
        self.calls.append(('forward', component))

    def prepare(self, component, confirmed):
        self.calls.append(('prepare', component, confirmed))
        if component == self.fail:
            raise ValueError('test component unavailable')
        return dict(action='reused', pid=123)

    def health(self, component):
        if component == 'device':
            return dict(domain='physical', connected=True, state_age_ms=40, owner='not_exported',
                        base=dict(stopped=True, fault=0), mechanism=dict(fault=0, attached=0))
        return dict(available=True, cameras=dict(material=dict(available=True),qr=dict(available=False)))


class ConnectionTests(unittest.TestCase):
    def test_forward_refused_is_a_connection_result_not_worker_crash(self):
        from paramiko import SSHException
        from types import SimpleNamespace
        handler=object.__new__(ForwardHandler)
        def reject(*args,**kwargs):raise SSHException('remote service restarting')
        handler.server=SimpleNamespace(transport=SimpleNamespace(open_channel=reject),remote_port=8091)
        handler.request=SimpleNamespace(getpeername=lambda:('127.0.0.1',12345))
        handler.handle()

    def test_start_is_inert(self):
        backend=FakeBackend(); manager=Connections(PROFILE,backend)
        self.assertEqual(manager.state()['phase'],'IDLE')
        self.assertEqual(backend.calls,[])

    def test_reuses_online_faulted_or_owned_device(self):
        for extra in ({},dict(owner='operator'),dict(base=dict(fault=2048)),dict(mechanism=dict(fault=105))):
            state=dict(domain='physical',connected=True,state_age_ms=42,**extra)
            self.assertEqual(decide('device',state,True,False),'reuse')

    def test_missing_or_stale_requires_confirmation_and_present_device(self):
        for state in (None,dict(connected=False),dict(domain='physical',connected=True,state_age_ms=300)):
            with self.assertRaises(ValueError):decide('device',state,True,False)
            self.assertEqual(decide('device',state,True,True),'start')
        with self.assertRaises(ValueError):decide('device',None,False,True)

    def test_partial_camera_and_later_second_camera(self):
        live=dict(cameras=dict(material=dict(available=True),qr=dict(available=False)))
        self.assertEqual(decide('vision',live,dict(material=True,qr=False)),'reuse')
        self.assertEqual(decide('vision',live,dict(material=True,qr=True)),'start')
        with self.assertRaises(ValueError):decide('vision',None,dict(material=False,qr=False))

    def test_component_failure_does_not_prevent_other_component(self):
        backend=FakeBackend('device');manager=Connections(PROFILE,backend)
        manager.run(ConnectCommand(component='all',password='TEST_SECRET'))
        self.assertEqual(manager.phase,'PARTIAL')
        self.assertTrue(manager.results['vision']['ok'])
        self.assertFalse(manager.results['device']['ok'])
        self.assertNotIn('TEST_SECRET',json.dumps(manager.state()))
        self.assertNotIn('not_exported',json.dumps(manager.state()))

    def test_single_component_does_not_restart_other(self):
        backend=FakeBackend();manager=Connections(PROFILE,backend)
        manager.run(ConnectCommand(component='vision'))
        self.assertEqual(backend.calls,[('ssh',),('forward','vision'),('prepare','vision',False)])

    def test_duplicate_prepare_is_rejected(self):
        manager=Connections(PROFILE,FakeBackend());manager.busy=True
        with self.assertRaises(ValueError):manager.start(ConnectCommand())

    def test_manager_requires_gateway_header(self):
        manager=Connections(PROFILE,FakeBackend())
        with TestClient(create_app(manager)) as client:
            self.assertEqual(client.post('/connect',json={}).status_code,403)
            self.assertEqual(client.post('/connect',json={},headers={'X-Workbench':'1','Origin':'https://evil.test'}).status_code,403)
            with patch.object(manager,'start',return_value=dict(status='CONNECTING')) as start:
                self.assertEqual(client.post('/connect',json={},headers={'X-Workbench':'1'}).status_code,200)
                self.assertEqual(start.call_count,1)

    def test_web_same_origin_and_no_password_echo(self):
        class Client:
            def request(self,path,body=None):
                if body:self.seen=body
                return dict(status='CONNECTING')
        backend=Client();app=FastAPI();app.include_router(router(backend))
        with TestClient(app) as client:
            self.assertEqual(client.post('/api/connections/connect',json={},headers={'X-Workbench':'1','Origin':'https://evil.test'}).status_code,403)
            reply=client.post('/api/connections/connect',json=dict(password='TEST_SECRET'),headers={'X-Workbench':'1','Origin':'http://testserver'})
            self.assertEqual(reply.status_code,200)
            self.assertNotIn('TEST_SECRET',reply.text)
            self.assertEqual(backend.seen['password'],'TEST_SECRET')


if __name__=='__main__':unittest.main()
