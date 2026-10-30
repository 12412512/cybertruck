import subprocess,tempfile,unittest
from pathlib import Path
from test_base_startup import BaseStartupTests,ROOT

class MicroAckTests(unittest.TestCase):
    def test_real_bus_replacement_and_old_negative(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'output/tests') as folder:
            p=Path(folder)
            for old in (False,True):
                source=(ROOT/'firmware/User/Board/base_port.c').read_text(encoding='utf-8')
                if old:
                    changed=source.replace('zero_id != zero || zero_action != action ||\n             memcmp(acknowledged_rpm, rpm, sizeof(acknowledged_rpm))','zero_id != zero || zero_action != action')
                    self.assertNotEqual(source,changed);source=changed
                (p/'port.c').write_text(source,encoding='utf-8')
                fixture=(ROOT/'tests/base_startup.c').read_text(encoding='utf-8').replace('"../firmware/User/Board/base_port.c"','"port.c"')
                (p/'base_fixture.c').write_text(fixture,encoding='utf-8')
                (p/'case.c').write_text((ROOT/'tests/micro_ack.c').read_text(encoding='utf-8').replace('"base_startup.c"','"base_fixture.c"'),encoding='utf-8')
                binary=p/('old.exe' if old else 'new.exe');BaseStartupTests.build(binary,p/'case.c')
                result=subprocess.run([str(binary)],capture_output=True,text=True)
                if old:self.assertNotEqual(result.returncode,0)
                else:self.assertEqual(result.returncode,0,result.stderr)
