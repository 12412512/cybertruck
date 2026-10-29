"""保护策略回归：缺失字段补齐、用户设置保留、重复执行无变化。"""

from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from keil_policy import enforce, OPTION_FLAGS


def project():
    return ET.fromstring('''<Project><Targets><Target><TargetName>Car</TargetName><TargetOption>
        <TargetCommonOption><PackID>keep-pack</PackID><ListingPath/></TargetCommonOption>
        <DllOption><SimDllName>sim.dll</SimDllName><TargetDllName>target.dll</TargetDllName></DllOption>
        <DebugOption><OPTHX/></DebugOption>
        <Utilities><Flash1><UpdateFlashBeforeDebugging>1</UpdateFlashBeforeDebugging></Flash1></Utilities>
        <TargetArmAds><Cads><Optim>6</Optim></Cads></TargetArmAds>
        </TargetOption><Groups><Group><GroupName>keep-group</GroupName></Group></Groups>
        </Target></Targets></Project>''')


class KeilPolicyTests(unittest.TestCase):
    def test_missing_debug_blocks_are_completed_in_schema_order(self):
        root = project()
        enforce(root)
        debug = root.find('.//DebugOption')
        self.assertEqual([node.tag for node in debug],
                         ['OPTHX', 'Simulator', 'Target', 'RunDebugAfterBuild', 'TargetSelection', 'SimDlls', 'TargetDlls'])
        self.assertEqual(debug.findtext('RunDebugAfterBuild'), '0')
        self.assertEqual(debug.findtext('Target/LoadApplicationAtStartup'), '0')
        self.assertEqual(root.findtext('.//UpdateFlashBeforeDebugging'), '0')

    def test_nonpolicy_user_settings_preserved_and_idempotent(self):
        root = project()
        enforce(root)
        before = ET.tostring(root)
        enforce(root)
        self.assertEqual(ET.tostring(root), before)
        self.assertEqual(root.findtext('.//PackID'), 'keep-pack')
        self.assertEqual(root.findtext('.//Optim'), '6')
        self.assertEqual(root.findtext('.//GroupName'), 'keep-group')

    def test_local_flags_disabled_without_overwriting_driver(self):
        root = project()
        options = ET.fromstring('''<ProjectOpt><Target><TargetName>Car</TargetName><TargetOption>
            <DebugOpt><pMon>keep-probe.dll</pMon><nTsel>5</nTsel><tLdApp>1</tLdApp><uSim>1</uSim><uTrg>0</uTrg></DebugOpt>
            </TargetOption></Target></ProjectOpt>''')
        enforce(root, options)
        for flag in OPTION_FLAGS:
            self.assertEqual(options.findtext('.//DebugOpt/' + flag), '0')
        self.assertEqual(root.findtext('.//TargetDlls/Driver'), 'keep-probe.dll')
        self.assertEqual(root.findtext('.//TargetSelection'), '5')
        self.assertEqual(root.findtext('.//Simulator/UseSimulator'), '1')
        self.assertEqual(root.findtext('.//DebugOption/Target/UseTarget'), '0')

    def test_invalid_target_rejected_before_mutation(self):
        root = project()
        root.find('.//TargetName').text = 'wrong-target'
        before = ET.tostring(root)
        with self.assertRaises(ValueError):
            enforce(root)
        self.assertEqual(ET.tostring(root), before)


if __name__ == '__main__':
    unittest.main(verbosity=2)
