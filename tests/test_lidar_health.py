"""雷达真实接口的只读握手离线验证；不创建Serial或访问设备。"""
import binascii
from pathlib import Path
import struct
import sys
import tempfile
import unittest

from fastapi import FastAPI
from fastapi.testclient import TestClient
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pi'))
from robot.lidar.protocol import cobs_encode,cobs_decode
from robot.lidar.health import Health
from robot.lidar_core import LidarCore,create_app,distinct_port
from robot.web.workbench import lidar_router


def event(kind,data,node=1):
    raw=struct.pack('<2sBBBHH',b'LR',6,node,kind,12,len(data))+data
    return cobs_encode(raw+struct.pack('<H',binascii.crc_hqx(raw,0xffff)))+b'\0'


def hello_data(zone=1,running=0):
    return struct.pack('<BBBBhhHBH',1,37,6,zone,150,2250,0,running,120)


def saved_data():
    raw=bytearray(33);struct.pack_into('<HI',raw,0,1,77);raw[6]=1
    struct.pack_into('<hhH',raw,11,150,2250,0)
    struct.pack_into('<hhH',raw,18,2250,2250,0)
    return bytes(raw)


def command(wire):return cobs_decode(wire[:-1])[4]


def ready():
    health=Health();assert command(health.step(0))==1
    health.feed(event(0x81,hello_data())+event(0x82,b'\1\0'),.01)
    assert command(health.step(.02))==0x13
    health.feed(event(0x82,b'\x13\0')+event(0x8e,saved_data()),.03)
    return health


class HealthTests(unittest.TestCase):
    def test_handshake_persisted_and_no_motion_authority(self):
        h=ready();v=h.view(.04)
        self.assertTrue(v['connected']);self.assertEqual(v['information']['firmware'],'1.37')
        self.assertEqual(v['persisted']['record_sequence'],77)
        self.assertFalse(v['scan_allowed']);self.assertFalse(v['real_map_registered'])
        self.assertFalse(v['authorizes_motion']);self.assertEqual(v['coverage'],'unknown')
        self.assertFalse(v['points']);self.assertFalse(v['obstacles'])

    def test_only_readonly_commands_and_bounded_heartbeat(self):
        h=ready();sent=[]
        for index in range(10):
            wire=h.step(.1+index*.1)
            if wire:sent.append(command(wire))
        self.assertEqual(sent,[2,2])
        for kind in (3,4,5,7,8,0x14,0x19):
            with self.subTest(kind=kind),self.assertRaises(ValueError):h.send(kind,.2)

    def test_requires_both_matching_ack_and_data(self):
        h=Health();h.step(0)
        h.feed(event(0x81,hello_data())+event(0x82,b'\x13\0'),.01)
        self.assertFalse(h.view(.02)['connected']);self.assertTrue(h.pending)
        self.assertEqual(command(h.step(.03)),2)
        h.step(1.5);self.assertTrue(h.fault);self.assertFalse(h.step(2))

    def test_unsolicited_storage_cannot_replace_requested_transaction(self):
        h=Health();h.step(0)
        h.feed(event(0x8e,saved_data())+event(0x81,hello_data())+event(0x82,b'\1\0'),.01)
        self.assertFalse(h.ready);self.assertEqual(command(h.step(.02)),0x13)

    def test_wrong_node_does_not_complete_handshake(self):
        h=Health();h.step(0)
        h.feed(event(0x81,hello_data(),2)+event(0x82,b'\1\0',2),.01)
        self.assertIsNone(h.info);self.assertEqual(h.last_rx,-1)
        h.step(1.5);self.assertTrue(h.fault)

    def test_split_frames_and_crc_fault_are_not_auto_recovered(self):
        h=Health();h.step(0);wire=event(0x81,hello_data())
        for byte in wire:h.feed(bytes([byte]),.01)
        self.assertIsNotNone(h.info)
        raw=bytearray(cobs_decode(event(0x82,b'\1\0')[:-1]));raw[-1]^=1
        h.feed(cobs_encode(raw)+b'\0',.02);self.assertTrue(h.fault)
        h.feed(event(0x82,b'\1\0'),.03);self.assertFalse(h.ready);self.assertFalse(h.step(.04))

    def test_rejection_and_device_error_latch(self):
        for wire in (event(0x82,b'\1\2'),event(0x8a,b'\1\0\0')):
            h=Health();h.step(0);h.feed(wire,.01)
            self.assertTrue(h.fault);self.assertFalse(h.step(.02))

    def test_stale_view_and_configuration_changes(self):
        h=ready();self.assertFalse(h.view(4)['connected'])
        h.feed(event(0x81,hello_data(2)),.1);self.assertTrue(h.fault)
        h=ready();h.step(6);self.assertTrue(h.fault)

    def test_running_information_does_not_start_or_stop_scanner(self):
        h=Health();h.step(0)
        h.feed(event(0x81,hello_data(running=1))+event(0x82,b'\1\0'),.01)
        self.assertEqual(command(h.step(.02)),0x13)
        self.assertFalse(h.view(.02)['scan_allowed']);self.assertEqual(set(h.sent),{1,2,0x13})

    def test_observed_137_extension_is_opaque_and_not_transaction_evidence(self):
        h=ready();h.feed(event(0x90,bytes.fromhex('54010a00')),.1)
        self.assertTrue(h.view(.1)['connected']);self.assertEqual(h.decoder.unknown,1)
        self.assertEqual(h.view(.1)['extensions'][0]['payload_hex'],'54010a00')
        self.assertFalse(h.view(.1)['real_map_registered']);self.assertFalse(h.view(.1)['scan_allowed'])
        h=Health();h.step(0);h.feed(event(0x90,b'abcd'),.1)
        self.assertTrue(h.fault);self.assertFalse(h.ready)

    def test_other_unknown_or_mixed_extension_is_not_hidden(self):
        for wire in (event(0x91,b'abcd'),event(0x90,b'abc'),
                     event(0x91,b'abcd')+event(0x90,b'abcd')):
            h=ready();h.feed(wire,.1);self.assertTrue(h.fault)

    def test_distinct_device_aliases_and_missing_ports_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            one=Path(temp)/'one';two=Path(temp)/'two';one.write_bytes(b'');two.write_bytes(b'')
            self.assertEqual(distinct_port(one,two),str(one))
            with self.assertRaises(ValueError):distinct_port(one,one)
            with self.assertRaises(ValueError):distinct_port(one,Path(temp)/'missing')

    def test_core_and_web_expose_only_get_state(self):
        core=LidarCore(clock=lambda:.04);core.health=ready()
        with TestClient(create_app(core)) as web:
            self.assertTrue(web.get('/state').json()['connected'])
            self.assertEqual(web.post('/command',json={'op':'scan'}).status_code,404)
        class Source:
            def request(self,path):return dict(domain='physical_lidar',connected=True,authorizes_motion=True,scan_allowed=True,real_map_registered=True)
        app=FastAPI();app.include_router(lidar_router(Source()))
        with TestClient(app) as web:
            result=web.get('/api/lidar/state').json()
            self.assertFalse(result['scan_allowed']);self.assertFalse(result['real_map_registered'])
            self.assertFalse(result['authorizes_motion'])
            self.assertEqual(web.post('/api/lidar/command',json={'op':'scan'}).status_code,404)


if __name__=='__main__':unittest.main()
