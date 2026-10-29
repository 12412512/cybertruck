"""人工故障恢复正反例；全部为合成会话、报文和文件。"""
import json
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import test_physical_mechanism as fixtures
import test_base_startup
from test_execution import ROOT
from robot.device.recovery import Recovery
from robot.device import protocol as p
from robot.mech_core import DeviceCore


class RecoveryTests(unittest.TestCase):
    def ready(self):
        link = fixtures.PhysicalMechanismTests().ready()
        link.firmware = 'GC-BASE-0.3.33'
        link.commissioning = True
        link.action_locked = link.stop_acknowledged = True
        link.actual.update(attached=1,config_mask=3,zero_version=1,zero_valid=0,fault=9,pwm_mask=3)
        link.base.update(stopped=True,fault=0x800)
        return link

    def args(self, link):
        return dict(confirmed=True,wiring_repaired=True,outputs_supported=True,
                    boot_id=str(link.boot),session_id=str(link.session),zero_version=1)

    def page(self):
        return struct.pack('<8I',1,0,0,0,64,0,40,0)

    def core(self, folder):
        core=DeviceCore(clock=lambda:1.01,runtime=folder,site_ready=True,commissioning=True)
        core.link=self.ready()
        return core

    def test_board_recovery_entry(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'output/tests') as folder:
            exe=Path(folder)/'recovery.exe'
            test_base_startup.BaseStartupTests.build(exe,ROOT/'tests/base_recovery.c')
            for case in range(10):
                r=subprocess.run([str(exe),str(case)],capture_output=True)
                self.assertEqual(r.returncode,0,r.stderr.decode(errors='replace'))

    def test_requires_fresh_explicit_confirmations_and_new_firmware(self):
        for field in ('confirmed','wiring_repaired','outputs_supported','boot_id','session_id','zero_version'):
            with self.subTest(field=field),tempfile.TemporaryDirectory() as folder:
                link=self.ready();args=self.args(link);args[field]=False
                with self.assertRaises(ValueError):Recovery(folder).start(link,args,1.01)
                self.assertFalse(link.commands)
        with tempfile.TemporaryDirectory() as folder:
            link=self.ready();link.firmware='GC-BASE-0.3.32'
            with self.assertRaises(ValueError):Recovery(folder).start(link,self.args(link),1.01)
            link.firmware='GC-BASE-0.3.33'
            with self.assertRaises(ValueError):Recovery(folder).start(link,self.args(link),1.3)

    def test_saves_before_any_request_and_no_motion_or_automatic_claim(self):
        with tempfile.TemporaryDirectory() as folder:
            core=self.core(folder);link=core.link;job=core.recovery
            job.start(link,self.args(link),1.01)
            self.assertTrue(job.path.exists());self.assertFalse(link.commands)
            job.advance(core,1.02)
            kind,data,rid=link.commands.popleft();self.assertEqual((kind,data),(38,b'\0'))
            link.inspections[rid]=dict(data=self.page().hex())
            job.advance(core,1.03)
            kind,data,rid=link.commands.popleft();self.assertEqual(kind,65)
            token,magic,flags,version=struct.unpack('<4I',data)
            self.assertTrue(token);self.assertEqual((magic,flags,version),(0x52454356,7,1))
            record=json.loads(job.path.read_text(encoding='utf-8'));self.assertTrue(record['can_pages'])
            self.assertEqual(link.actual['pwm_mask'],3) # 主机未伪造已关闭。
            link.inspections[rid]=dict(result=0)
            job.advance(core,1.04);self.assertEqual(job.status,'REBOOT')
            self.assertEqual(link.step(1.1),b'')
            job.advance(core,1.6);self.assertIsNot(core.link,link)
            new=self.ready();new.boot=link.boot+1;new.action_locked=new.stop_acknowledged=False
            new.actual.update(fault=0,pwm_mask=0,zero_valid=0,axis_id=0,servo_id=0,pose_id=0)
            new.base.update(fault=0);new.actual_at=new.base_at=1.6;core.link=new
            job.advance(core,1.61);self.assertEqual(job.status,'ATTACH')
            self.assertEqual(new.commands.popleft()[0],48)
            job.advance(core,1.62)
            new.actual_at=new.base_at=2.7
            job.advance(core,2.7)
            self.assertEqual(job.status,'DONE');self.assertFalse(core.owner or core.token)
            self.assertFalse(new.commands);self.assertFalse(new.actual['zero_valid'])

    def test_failed_archive_never_sends(self):
        with tempfile.TemporaryDirectory() as folder:
            link=self.ready();job=Recovery(folder)
            with patch.object(job,'save',side_effect=OSError('disk full')):
                with self.assertRaises(ValueError):job.start(link,self.args(link),1.01)
            self.assertFalse(job.active);self.assertFalse(link.commands)

    def test_bad_capture_or_rejected_reset_never_retries(self):
        for bad in (True,False):
            with self.subTest(bad=bad),tempfile.TemporaryDirectory() as folder:
                core=self.core(folder);job=core.recovery;link=core.link
                job.start(link,self.args(link),1.01);job.advance(core,1.02)
                _,_,rid=link.commands.popleft();link.inspections[rid]=dict(data=('00' if bad else self.page().hex()))
                error=job.advance(core,1.03)
                if not bad:
                    _,_,rid=link.commands.popleft();link.inspections[rid]=dict(result=3)
                    error=job.advance(core,1.04)
                self.assertTrue(error);self.assertEqual(job.status,'FAILED')
                self.assertFalse(link.commands);self.assertIs(core.link,link)

    def test_stop_cancels_and_can_send_stop_during_reboot_wait(self):
        with tempfile.TemporaryDirectory() as folder:
            core=self.core(folder);job=core.recovery;link=core.link
            job.start(link,self.args(link),1.01);link.recovery_wait=True
            core.stop_control('cancel','test','pc')
            self.assertEqual(job.status,'CANCELLED');self.assertFalse(link.recovery_wait)
            self.assertTrue(link.action_locked);self.assertIs(core.link,link)
            self.assertEqual(p.Decoder().feed(link.step(1.02))[0].kind,p.STOP)

    def test_repeated_request_has_one_archive_and_blocks_claim(self):
        with tempfile.TemporaryDirectory() as folder:
            core=self.core(folder);req=dict(op='recover',client='pc',request_id='a',args=self.args(core.link))
            first=core.command(req);self.assertEqual(core.command(req),first)
            self.assertEqual(len(list(Path(folder).glob('fault_recovery/*.json'))),1)
            with self.assertRaises(ValueError):core.command(dict(op='claim',client='pc',request_id='claim'))

    def test_does_not_accept_same_boot_or_persistent_fault(self):
        for changed,fault in ((False,0),(True,0x800)):
            with self.subTest(changed=changed),tempfile.TemporaryDirectory() as folder:
                core=self.core(folder);job=core.recovery
                job.start(core.link,self.args(core.link),1.01);job.status='VERIFY'
                if changed:core.link.boot+=1
                core.link.base['fault']=fault;core.link.actual['fault']=0
                self.assertTrue(job.advance(core,1.02));self.assertEqual(job.status,'FAILED')

    def test_inspection_reply_and_reset_reply_use_matching_original_request(self):
        link=self.ready();link.operation='archive'
        req=p.Packet(38,link.boot,link.session,1,250,b'\0')
        reply=p.Packet(166,link.boot,link.session,1,0,b'\0'+self.page())
        self.assertTrue(link.custom_reply(reply,req,1.01))
        self.assertEqual(link.inspections['archive']['data'],self.page().hex())
        link.operation='reset';req=p.Packet(65,link.boot,link.session,2,250,b'')
        reply=p.Packet(193,link.boot,link.session,2,0,b'\0'+struct.pack('<I',3))
        link.custom_reply(reply,req,1.02);self.assertEqual(link.inspections['reset']['result'],3)

    def test_complete_first_error_history_is_saved_before_reset(self):
        with tempfile.TemporaryDirectory() as folder:
            core=self.core(folder);job=core.recovery;link=core.link
            job.start(link,self.args(link),1.01);job.advance(core,1.02)
            _,_,rid=link.commands.popleft()
            header=lambda page:struct.pack('<8I',1,1,1,1,64,page,40,0)
            first=[17,0x1000,1,4,2,3,0x500,3,0,0,0,0,0,0,0,1]
            link.inspections[rid]=dict(data=(header(0)+struct.pack('<16I',*first)).hex())
            job.advance(core,1.03)
            kind,page,rid=link.commands.popleft();self.assertEqual((kind,page),(38,b'\1'))
            event=[1,17,5,0x500,3,0,0,2,1,4]
            link.inspections[rid]=dict(data=(header(1)+struct.pack('<10I',*event)).hex())
            job.advance(core,1.04)
            self.assertEqual(link.commands[0][0],65)
            saved=json.loads(job.path.read_text(encoding='utf-8'))
            self.assertTrue(saved['can_capture']['complete'])
            self.assertEqual(saved['can_capture']['first']['tick_ms'],17)
            self.assertEqual(saved['can_capture']['history'][0]['kind_name'],'ERROR')

    def test_missing_reset_ack_or_timeout_does_not_issue_a_second_reset(self):
        with tempfile.TemporaryDirectory() as folder:
            core=self.core(folder);job=core.recovery;link=core.link
            job.start(link,self.args(link),1.01);job.advance(core,1.02)
            _,_,rid=link.commands.popleft();link.inspections[rid]=dict(data=self.page().hex())
            job.advance(core,1.03);self.assertEqual(link.commands.popleft()[0],65)
            self.assertTrue(job.advance(core,2.1));self.assertEqual(job.status,'FAILED')
            self.assertFalse(link.commands);self.assertIs(core.link,link)
