"""STOP后只读保活、停止来源和旧页面隔离；全部使用合成协议，无设备IO。"""
import struct
import unittest
import test_physical_mechanism as fixtures
from robot.device import protocol as p
from robot.device.mechanism import FIELDS
from robot.device.execution import STATUS_DATA
from robot.mech_core import DeviceCore


class Replies:
    def __init__(self):
        self.rx = b''
        self.sent = []

    def write(self, wire):
        request = p.Decoder().feed(wire)[0]
        self.sent.append(request)
        data = b''
        if request.kind == 58:
            state = dict.fromkeys(FIELDS, 0)
            state.update(schema=1, attached=1, fault=6, config_mask=3, base_stopped=1)
            data = struct.pack('<28I', *(state[key] for key in FIELDS))
        elif request.kind == 35:
            data = STATUS_DATA.pack(6, 0x800, 0, 0, 0, 1, 1234, 0, 9, 99, 0, *([0.]*9))
        elif request.kind == p.STATE:
            data = p.STATE_DATA.pack(1234, 0x800, 7, 10, 20, 0, 0)
        elif request.kind == p.DIAG:
            data = struct.pack('<II', 0, 1)
        else:
            assert request.kind in (p.STOP, p.HEART)
        self.rx += p.Packet(request.kind|128, request.boot, request.session, request.request,
                            0, bytes([p.OK])+data).encode()
        return len(wire)

    def read(self, _):
        value, self.rx = self.rx, b''
        return value


class StopReadonlyTests(unittest.TestCase):
    def ready(self):
        return fixtures.PhysicalMechanismTests().ready()

    def test_stop_keeps_polling_without_open_or_motion_and_keeps_real_fault(self):
        now = [1.01]
        core = DeviceCore(clock=lambda:now[0], site_ready=True)
        core.link = self.ready()
        core.link.submit('queued', 'move', dict(axis='lift',target_m=.002,zero_version=1), now[0])
        core.command(dict(op='stop', client='operator', request_id='stop1', args=dict(source='panel-stop-button')))
        transport = Replies()
        identity = core.link.boot, core.link.session
        for _ in range(160):
            core.tick(transport)
            now[0] += .02
        state = core.state()
        self.assertTrue(state['connected'])
        self.assertTrue(state['action_locked'])
        self.assertTrue(state['stop_acknowledged'])
        self.assertFalse(state['latched'])
        self.assertEqual((core.link.boot,core.link.session),identity)
        self.assertEqual(state['base']['fault'],0x800)
        self.assertEqual(state['mechanism']['fault'],6)
        self.assertEqual(state['safety_events'][-1]['source'],'panel-stop-button')
        self.assertEqual(sum(r.kind==p.STOP for r in transport.sent),1)
        self.assertGreater(sum(r.kind==58 for r in transport.sent),5)
        self.assertGreater(sum(r.kind==35 for r in transport.sent),5)
        self.assertNotIn(p.OPEN,[r.kind for r in transport.sent])
        with self.assertRaises(ValueError):
            core.command(dict(op='claim',client='operator',request_id='claim-again'))

    def test_lease_still_stops_at_original_deadline(self):
        core=DeviceCore(clock=lambda:2.)
        core.link=self.ready();core.owner='a';core.token='secret';core.until=1.99
        transport=Replies();core.tick(transport)
        self.assertEqual(transport.sent[0].kind,p.STOP)
        self.assertFalse(core.token)
        self.assertEqual(core.safety_events[-1]['source'],'lease-expired')
        self.assertTrue(core.link.action_locked)

    def test_delayed_release_from_old_page_cannot_stop_new_owner(self):
        core=DeviceCore(clock=lambda:1.01)
        core.link=self.ready();core.owner='new';core.token='new-token';core.until=2.
        result=core.command(dict(op='release',client='old',token='old-token',request_id='late',args=dict(source='page-hidden')))
        self.assertEqual(result['status'],'IGNORED_OLD_OWNER')
        self.assertEqual(core.owner,'new')
        self.assertFalse(core.link.action_locked)
        self.assertFalse(core.safety_events)
        result=core.command(dict(op='release',client='new',token='new-token',request_id='current',args=dict(source='page-hidden')))
        self.assertEqual(result['status'],'STOP_REQUESTED')
        self.assertTrue(core.link.action_locked)
        self.assertEqual(core.safety_events[-1]['source'],'page-hidden')
        self.assertNotIn('token',str(list(core.safety_events)))

    def test_stop_without_ack_is_not_reported_connected_forever(self):
        link=self.ready();link.emergency('manual stop')
        first=p.Decoder().feed(link.step(1.01))[0]
        self.assertEqual(first.kind,p.STOP)
        self.assertFalse(link.stop_acknowledged)
        self.assertEqual(link.step(1.27),b'')
        self.assertTrue(link.latched)
        self.assertFalse(link.view(1.28)['connected'])
        self.assertEqual(p.Decoder().feed(link.step(1.28))[0].kind,p.STOP)
        self.assertEqual(link.step(5),b'')

    def test_repeated_stop_does_not_reset_polling_or_clear_faults(self):
        link=self.ready();link.base['fault']=0x800
        link.emergency('first');transport=Replies()
        request=p.Decoder().feed(link.step(1.01))[0]
        link.feed(p.Packet(request.kind|128,request.boot,request.session,request.request,0,b'\0').encode(),1.02)
        link.emergency('second')
        self.assertTrue(link.stop_acknowledged)
        self.assertEqual(link.base['fault'],0x800)
        self.assertNotEqual(p.Decoder().feed(link.step(1.03))[0].kind,p.STOP)


if __name__=='__main__':
    unittest.main()
