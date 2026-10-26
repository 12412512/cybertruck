"""三舵机归零：只验证合成协议，首次输出不伪装限速或真实到位。"""
import struct
import unittest
from fastapi.testclient import TestClient
import test_physical_mechanism as fixtures
from robot.device.mechanism import payload
from robot.device import protocol as p
from robot.mech_core import DeviceCore, create_app


class HomeTests(unittest.TestCase):
    def ready(self):
        link=fixtures.PhysicalMechanismTests().ready()
        link.firmware='GC-BASE-0.3.34';link.commissioning=link.manual_debug=True
        link.home=dict(schema=1,id=0,state=0,zero_version=1)
        link.home_at=1.
        return link

    def test_wire_requires_explicit_zero_and_matching_reference_version(self):
        args=dict(confirmed=True,safe_zero=True,zero_version=1)
        kind,data=payload('home',args,dict(servo_id=3,home_id=8),7)
        self.assertEqual(kind,66);self.assertEqual(struct.unpack('<IIIB',data),(7,1,9,1))
        args.pop('safe_zero')
        with self.assertRaises(ValueError):payload('home',args,{},7)
        args['home_id']=9
        kind,data=payload('home_confirm',args,{},8)
        self.assertEqual((kind,struct.unpack('<IIIB',data)),(67,(8,1,9,1)))

    def test_start_only_queues_and_confirm_waits_for_hardware_status(self):
        link=self.ready();before=link.actual.copy()
        link.submit('home','home',dict(confirmed=True,safe_zero=True,zero_version=1),1.01)
        self.assertEqual(link.commands[0][0],66);self.assertEqual(link.actual,before)
        link.commands.clear();link.home.update(id=1,state=2)
        for action,args in [('move',dict(axis='arm',target_rad=0,zero_version=1)),
                            ('home',dict(confirmed=True,safe_zero=True,zero_version=1)),
                            ('home_confirm',dict(confirmed=True,home_id=1,zero_version=1))]:
            with self.assertRaises(ValueError):link.submit('bad',action,args,1.02)
        link.home['state']=3
        link.submit('confirm','home_confirm',dict(confirmed=True,home_id=1,zero_version=1),1.03)
        self.assertEqual(link.commands[0][0],67)

    def test_firmware_manual_and_freshness_guards(self):
        for mutate in (lambda s:setattr(s,'firmware','GC-BASE-0.3.33'),
                       lambda s:setattr(s,'manual_debug',False),
                       lambda s:setattr(s,'home_at',.5),
                       lambda s:s.home.update(zero_version=2),
                       lambda s:s.base.update(fault=2048)):
            link=self.ready();mutate(link)
            with self.assertRaises(ValueError):link.submit('home','home',dict(confirmed=True,safe_zero=True,zero_version=1),1.01)
            self.assertFalse(link.commands)

    def test_status_decode_poll_and_reference_is_not_assumed(self):
        link=self.ready();kinds=[]
        for i in range(4):kinds.append(link.extra_request(1.01+i*.031)[0])
        self.assertEqual(kinds,[35,58,64,68])
        req=p.Packet(68,link.boot,link.session,1,250,b'')
        data=struct.pack('<8I',1,1000,3,3,0,1,7,400)
        reply=p.Packet(196,link.boot,link.session,1,0,b'\0'+data)
        link.custom_reply(reply,req,1.14)
        self.assertEqual(link.home['state'],3);self.assertEqual(link.actual['home_id'],3)
        self.assertEqual(link.view(1.14)['home']['id'],3)
        self.assertIsNone(link.view(1.4)['home'])

    def test_wrong_or_early_confirm_does_not_send(self):
        link=self.ready();link.home.update(id=2,state=3)
        with self.assertRaises(ValueError):link.submit('confirm','home_confirm',dict(confirmed=True,home_id=1,zero_version=1),1.01)
        self.assertFalse(link.commands)

    def test_same_request_via_api_only_queues_once_and_stop_never_rehomes(self):
        core=DeviceCore(clock=lambda:1.01,site_ready=True,commissioning=True);core.link=self.ready()
        core.owner='pc';core.token='lease';core.until=2.
        body=dict(op='mechanism',client='pc',token='lease',request_id='home1',
                  args=dict(action='home',confirmed=True,safe_zero=True,zero_version=1))
        with TestClient(create_app(core)) as client:
            first=client.post('/command',json=body);self.assertEqual(first.status_code,200)
            self.assertEqual(client.post('/command',json=body).json(),first.json())
            self.assertEqual(len(core.link.commands),1)
            client.post('/command',json=dict(op='stop',client='pc',request_id='stop'))
        self.assertFalse(core.link.commands);self.assertTrue(core.link.action_locked)
        self.assertFalse(core.link.manual_debug)
