"""PC0真实超时记录解码和倒计时；不创建串口，不续期板端准备。"""
import struct
import unittest

from robot.device.mechanism import MechanismLink
from robot.device.protocol import Packet


class StartStatusTests(unittest.TestCase):
    def reply(self, link, kind, phase, now, sent=None, ticket=1):
        values=[3,phase,ticket,10,int(phase==4),0,
                3 if phase==5 else 0,361069 if phase==5 else 0,
                0,361069,360965,361058,361057,
                360991,361019,361045,360965,360987,361013,361040,361068,
                0,1,1,1,0,0]
        return link.custom_reply(Packet(kind|128,123,456,1,0,b'\0'+struct.pack('<27I',*values)),
                                 Packet(kind,123,456,1,250,b''),now,sent)

    def test_timeout_retains_frozen_board_diagnostic(self):
        link=MechanismLink();link.boot=123
        self.reply(link,59,2,10.1,10.)
        self.reply(link,60,5,70.01,70.)
        self.assertEqual(link.start_event['reason'],3)
        self.assertEqual(link.start_event['aborted_ms'],361069)
        self.assertEqual(link.start_event['guard_mask'],0)
        self.assertEqual(link.start_event['position4_ms'],360965)
        view=link.view(70.02)['physical_start']
        self.assertEqual(view['phase'],5);self.assertIsNone(view['remaining_ms'])
        self.assertNotIn('deadline_s',view);self.assertNotIn('at',view)

    def test_query_and_duplicate_prepare_never_extend_countdown(self):
        link=MechanismLink();link.boot=123
        self.reply(link,59,1,10.1,10.)
        self.assertEqual(link.view(11.)['physical_start']['remaining_ms'],59000)
        self.reply(link,60,2,39.1,39.)
        self.assertEqual(link.start_event['deadline_s'],70.)
        self.reply(link,59,2,40.1,40.)
        self.assertEqual(link.start_event['deadline_s'],70.)
        self.assertEqual(link.view(69.)['physical_start']['remaining_ms'],1000)
        self.reply(link,59,1,100.1,100.,ticket=2)
        self.assertEqual(link.start_event['deadline_s'],160.)

    def test_inconsistent_fired_state_rejected(self):
        link=MechanismLink();link.boot=123
        payload=b'\0'+struct.pack('<27I',3,2,1,10,1,*([0]*22))
        link.custom_reply(Packet(188,123,456,1,0,payload),Packet(60),1.,.9)
        self.assertTrue(link.latched);self.assertFalse(link.start_event)
