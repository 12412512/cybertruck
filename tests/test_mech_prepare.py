"""正常停止重新准备：全合成状态，不连接串口或设备。"""
import struct
import unittest
import test_physical_mechanism as fixtures
from robot.device import protocol as p
from robot.mech_core import DeviceCore

class PrepareTests(unittest.TestCase):
    def ready(self):
        link=fixtures.PhysicalMechanismTests().ready()
        link.firmware='GC-BASE-0.3.26';link.commissioning=True
        link.action_locked=True;link.stop_acknowledged=True
        link.actual.update(attached=1,fault=6,config_mask=3,zero_valid=0,pwm_mask=0)
        return link

    def args(self, link):
        return dict(confirmed=True,boot_id=str(link.boot),session_id=str(link.session),zero_version=1)

    def test_same_session_explicit_prepare_only_unlocks_after_board_accept(self):
        link=self.ready();identity=link.boot,link.session
        result=link.prepare('prepare1',self.args(link),1.01)
        self.assertEqual(result['status'],'QUEUED');self.assertTrue(link.action_locked)
        kind,data,rid=link.commands.popleft();self.assertEqual(kind,61)
        self.assertEqual(struct.unpack('<IIB',data),(1,1,1))
        link.operation=rid;request=p.Packet(61,link.boot,link.session,11,250,data)
        reply=p.Packet(189,link.boot,link.session,11,0,bytes([p.OK])+struct.pack('<I',0))
        link.custom_reply(reply,request,1.02)
        self.assertFalse(link.action_locked);self.assertFalse(link.view(1.02)['connected'])
        self.assertEqual(identity,(link.boot,link.session))
        self.assertFalse(link.commands);self.assertEqual(link.results[0]['status'],'ACCEPTED')

    def test_fault_stale_pwm_confirmation_identity_and_old_firmware_remain_blocked(self):
        for case in ('can','mech','stale','pwm','stop_ack','confirm','boot','session','version','old','readonly'):
            link=self.ready();args=self.args(link)
            if case=='can':link.base['fault']=2048
            elif case=='mech':link.actual['fault']=2
            elif case=='stale':link.actual_at=0
            elif case=='pwm':link.actual['pwm_mask']=1
            elif case=='stop_ack':link.stop_acknowledged=False
            elif case=='confirm':args['confirmed']=False
            elif case in ('boot','session'):args[case+'_id']='wrong'
            elif case=='version':args['zero_version']=2
            elif case=='old':link.firmware='GC-BASE-0.3.25'
            elif case=='readonly':link.commissioning=False
            with self.subTest(case=case),self.assertRaises(ValueError):link.prepare('p',args,1.01)
            self.assertFalse(link.commands);self.assertTrue(link.action_locked)

    def test_stop_preempts_pending_prepare_without_unlock(self):
        for submitted in (False,True):
            link=self.ready();link.prepare('p',self.args(link),1.01)
            if submitted:
                kind,data,rid=link.commands.popleft();link.operation=rid
                link.pending=(p.Packet(kind,link.boot,link.session,3,250,data),1.01)
            link.emergency('new stop')
            self.assertFalse(link.commands);self.assertIsNone(link.pending)
            self.assertTrue(link.action_locked);self.assertTrue(link.stop_queued)
            self.assertFalse(link.stop_acknowledged)

    def test_core_explicit_request_dedup_and_no_auto_claim(self):
        core=DeviceCore(clock=lambda:1.01,site_ready=True,commissioning=True);core.link=self.ready()
        request=dict(op='prepare',client='pc',request_id='p',args=self.args(core.link))
        first=core.command(request);self.assertEqual(core.command(request),first)
        self.assertEqual(len(core.link.commands),1);self.assertFalse(core.owner);self.assertFalse(core.token)
        request['args']['confirmed']=False
        with self.assertRaises(ValueError):core.command(request)
