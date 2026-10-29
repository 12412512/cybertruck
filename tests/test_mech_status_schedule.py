"""主机状态轮询与已到达串口回复回放；不连接设备。"""
import unittest,struct
import test_mech_sequence as fixtures
from test_mech_stop_readonly import Replies
from robot.mech_core import DeviceCore
from robot.device import protocol as p
from robot.device.execution import STATUS_DATA

class StatusScheduleTests(unittest.TestCase):
    def delayed_core(self):
        now=[1.]
        core=DeviceCore(clock=lambda:now[0],site_ready=True,commissioning=True)
        core.link=fixtures.SequenceTests().ready()
        core.owner='operator';core.token='token';core.until=1.1
        class DelayedLock:
            def __enter__(self):now[0]=1.2
            def __exit__(self,*args):pass
        core.lock=DelayedLock()
        return core

    def test_lock_wait_cannot_renew_expired_lease(self):
        core=self.delayed_core()
        with self.assertRaisesRegex(ValueError,'控制权已失效'):
            core.command(dict(op='renew',client='operator',token='token',request_id='late'))
        self.assertEqual(core.until,1.1)
        self.assertFalse(core.link.commands)

    def test_tick_checks_lease_after_lock_wait(self):
        core=self.delayed_core()
        core.tick(Replies())
        self.assertFalse(core.token)
        self.assertEqual(core.safety_events[-1]['source'],'lease-expired')

    def test_view_reports_oldest_required_feedback(self):
        link=fixtures.SequenceTests().ready()
        link.actual_at=1.19;link.base_at=1.0
        state=link.view(1.2)
        self.assertTrue(state['connected'])
        self.assertEqual(state['state_age_ms'],200)
        self.assertEqual(state['feedback_age_ms']['base'],200)
        self.assertFalse(link.view(1.251)['connected'])
        link.base_at=-1
        self.assertIsNone(link.view(1.2)['state_age_ms'])
        self.assertFalse(link.view(1.2)['connected'])

    def replay(self,firmware,delay):
        link=fixtures.SequenceTests().ready();link.firmware=firmware
        link.home_at=link.last_heart=1.;delivery=None;maximum=0.;commands=0;next_command=1.
        for i in range(8000):
            now=1+i*.005
            if delivery and now>=delivery[0]:
                kind=delivery[1];link.pending=None;delivery=None
                for code,name in ((35,'base_at'),(58,'actual_at'),(64,'group_at'),(68,'home_at'),(72,'handoff_at')):
                    if kind==code:setattr(link,name,now)
                if kind==63:link.operation=None
            if now>=next_command:
                link.commands.append((63,b'group',str(i)));next_command=now+.15
            wire=link.step(now)
            if wire:
                packet=p.Decoder().feed(wire)[0]
                if packet.kind==63:commands+=1
                delivery=(now+delay+(.015 if packet.kind in (69,70) else 0),packet.kind)
            maximum=max(maximum,now-link.base_at,now-link.actual_at,now-link.group_at)
        return maximum,commands

    def test_command_and_diagnostic_load_keeps_250ms_without_raising_limit(self):
        for delay in (.020,.025,.030):
            with self.subTest(reply_delay_s=delay):
                old,_=self.replay('GC-BASE-0.3.40',delay)
                current,count=self.replay('GC-BASE-0.3.41',delay)
                self.assertGreater(old,.25)
                self.assertLess(current,.25)
                self.assertGreater(count,100) # 关键轮询不能靠永不发命令来过测试。

    def test_handoff_status_poll_keeps_critical_feedback_and_commands(self):
        for delay in (.020,.025,.030):
            with self.subTest(reply_delay_s=delay):
                maximum,count=self.replay('GC-BASE-0.3.45',delay)
                self.assertLess(maximum,.25)
                self.assertGreater(count,100)

    def test_actual_stale_feedback_still_fails_with_specific_cause(self):
        from robot.device.mech_sequence import Sequence,record
        fixture=fixtures.SequenceTests();link=fixture.ready();link.firmware='GC-BASE-0.3.41'
        seq=Sequence();seq.start(record('test',fixture.rows(),1),'ordered',link,1.01)
        link.actual_at=link.group_at=1.2
        error=seq.advance(link,1.254)
        self.assertIn('底盘状态过期254ms',error)
        self.assertFalse(link.commands)

    def test_critical_refresh_preempts_command_and_diagnostics(self):
        link=fixtures.SequenceTests().ready();link.firmware='GC-BASE-0.3.41';link.home_at=1.
        link.base_at=.81;link.actual_at=.85;link.group_at=.89
        link.commands.append((63,b'goal','one'))
        self.assertEqual(link.extra_request(1.01)[0],35)
        self.assertEqual(len(link.commands),1)
        link.base_at=1.02
        self.assertEqual(link.extra_request(1.03)[0],58)
        self.assertEqual(len(link.commands),1)

    def test_ready_reply_is_consumed_before_sequence_age_check(self):
        now=[1.254];core=DeviceCore(clock=lambda:now[0],site_ready=True,commissioning=True)
        core.link=fixtures.SequenceTests().ready();link=core.link;link.firmware='GC-BASE-0.3.41'
        link.base_at=1.;link.actual_at=link.group_at=1.2
        req=p.Packet(35,link.boot,link.session,50,250,b'')
        link.pending=(req,1.2)
        data=STATUS_DATA.pack(0,0,0,0,0,1,1234,0,0,0,0,*([0.]*9))
        reply=p.Packet(163,link.boot,link.session,50,0,bytes([p.OK])+data).encode()
        class Ready(Replies):
            def __init__(self):super().__init__();self.rx=reply
            @property
            def in_waiting(self):return len(self.rx)
        transport=Ready();observed=[]
        core.sequence.advance=lambda state,tick:observed.append(tick-state.base_at) or ''
        core.tick(transport)
        self.assertEqual(observed,[0.])
        self.assertTrue(link.bound)

    def test_expired_pending_reply_does_not_become_new_permission(self):
        now=[1.5];core=DeviceCore(clock=lambda:now[0]);core.link=fixtures.SequenceTests().ready()
        link=core.link;link.pending=(p.Packet(35,link.boot,link.session,50,250,b''),1.)
        class Ready(Replies):
            @property
            def in_waiting(self):return len(self.rx)
        transport=Ready();transport.rx=p.Packet(163,link.boot,link.session,50,0,bytes([p.OK])+b'late').encode()
        core.tick(transport)
        self.assertTrue(link.latched)
        self.assertEqual(link.base_at,1.)
