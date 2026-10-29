"""真实C微调器+独立起步迟滞模型；仅离线，不把合成结果当实测。"""
import json
import math
import subprocess
import sys
import time
import unittest
from test_micro_dynamics import DelayedMicro, ROOT


def bursts(model):
    runs=[];active=False
    for tick,out in model.commands:
        moving=any(out['rpm'])
        if moving and not active:runs.append([tick,tick])
        if moving:runs[-1][1]=tick
        active=moving
    return runs


def delayed_start(model, severity=1.):
    """在独立Plant输入侧加轮间迟滞；时长用于反例，不声称已经标定。"""
    plant=model.plant;integrate=plant.integrate
    requested=[0.]*4;since=[0]*4
    def advance(disturbance=False):
        desired=plant.targets[:]
        for i,target in enumerate(desired):
            if target != requested[i]:
                # 同向加速不重新计算起步等待；停车/换向才新建。
                if not target or target*requested[i]<=0:since[i]=plant.ms
                requested[i]=target
            delay=([180,300,250,400][i] if abs(target)<1.5 else [25,50,40,60][i])*severity
            if target and plant.ms-since[i]<delay:plant.targets[i]=0
        integrate(disturbance)
        plant.targets[:]=desired
    plant.integrate=advance


class MicroSmoothTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.library=ROOT/'output/tests'/f'micro-smooth-{time.time_ns()}.dll'
        subprocess.run([sys.executable,str(ROOT/'tools/build_odom.py'),'--output',str(cls.library)],check=True,capture_output=True)

    def test_start_delay_and_four_small_directions(self):
        reports=[]
        for severity in (.25,.5,1):
            for axis,mm in [('x',5),('x',-5),('y',5),('y',-5),('y',1),('y',-1)]:
                with self.subTest(severity=severity,axis=axis,mm=mm):
                    model=DelayedMicro(self.library,axis,mm,period=160,tau=.2)
                    delayed_start(model,severity)
                    result=model.run(11000)
                    self.assertEqual(result['state'],'COMPLETE',result['output'])
                    self.assertLessEqual(abs(getattr(model.plant,axis)-mm/1000),.001)
                    self.assertLessEqual(abs(model.plant.yaw),math.radians(.2))
                    self.assertLessEqual(result['peak_rpm'],2)
                    reports.append(dict(severity=severity,axis=axis,mm=mm,result=result['output'],elapsed_ms=result['elapsed_ms'],bursts=len(bursts(model))))
        (ROOT/'output/tests/micro_start_delay_0349.json').write_text(json.dumps(reports,indent=2),encoding='utf-8')

    def test_longer_segment_does_not_relax_feedback_protection(self):
        model=DelayedMicro(self.library)
        for _ in range(15):model.advance(False)
        model.drop.add((1,model.calibration.address[0]));started=model.elapsed_ms
        result=model.run(11000)
        self.assertEqual(result['state'],'FAILED')
        self.assertEqual(result['output']['fault'],2)
        self.assertLessEqual(model.elapsed_ms-started,260)
        self.assertEqual(result['output']['rpm'],[0]*4)

    def test_no_repeat_start_before_zero_ack_and_stability(self):
        model=DelayedMicro(self.library,period=200,tau=.35)
        model.run(11000)
        runs=bursts(model)
        for first,second in zip(runs,runs[1:]):self.assertGreaterEqual(second[0]-first[1],150)
        # 至少一段超过旧200ms，且幅值四轮相等；没有中途反向追旧反馈。
        self.assertGreater(max(b-a for a,b in runs),250)
        for _,out in model.commands:self.assertEqual(len(set(map(abs,out['rpm']))),1)


if __name__=='__main__':unittest.main()
