"""联调版本归档删除：临时目录与合成设备，不发送真实UART或删除用户记录。"""

import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from robot.mech_core import DeviceCore
from robot.device.mech_sequence import record
import test_mech_sequence as sequence_fixture


class SequenceDeleteTests(unittest.TestCase):
    def setUp(self):
        self.folder=tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.core=DeviceCore(clock=lambda:1.1,runtime=self.folder.name,site_ready=True,commissioning=True)
        self.core.link=sequence_fixture.SequenceTests().ready()
        self.first=record('测试',sequence_fixture.SequenceTests().rows(),1)
        self.second=record('测试',sequence_fixture.SequenceTests().rows(),2)
        self.core.sequences=[self.first,self.second]
        self.before_commands=list(self.core.link.commands)

    def request(self, **change):
        value=dict(op='sequence_delete',client='test',request_id='delete1',args=dict(id='测试',revision=1,sha256=self.first['sha256'],confirmed=True))
        value.update(change)
        return value

    def test_delete_single_version_keeps_rows_and_never_sends_uart(self):
        reply=self.core.command(self.request())
        self.assertEqual(reply['status'],'ARCHIVED_VERSION')
        self.assertEqual(self.core.visible_sequences(),[self.second])
        self.assertEqual(self.core.sequences,[self.first,self.second])
        self.assertEqual(list(self.core.link.commands),self.before_commands)
        self.assertEqual(self.core.sequence_archive[0]['record'],self.first)
        self.assertFalse(reply['action_sent'])

    def test_repeated_request_is_idempotent_and_conflict_rejected(self):
        value=self.request();first=self.core.command(value)
        self.assertEqual(self.core.command(value),first)
        self.assertEqual(len(self.core.sequence_archive),1)
        changed=copy.deepcopy(value);changed['args']['revision']=2
        with self.assertRaises(ValueError):self.core.command(changed)

    def test_wrong_hash_revision_or_confirmation_cannot_delete(self):
        for values in (dict(sha256='wrong'),dict(revision=True),dict(revision=3),dict(confirmed=False)):
            value=self.request();value['args'].update(values)
            with self.assertRaises(ValueError):self.core.command(value)
        self.assertEqual(self.core.visible_sequences(),[self.first,self.second])

    def test_running_or_foreign_owner_rejected(self):
        self.core.sequence.status='RUNNING'
        with self.assertRaises(ValueError):self.core.command(self.request())
        self.core.sequence.status='IDLE';self.core.token='token';self.core.owner='other';self.core.until=3.
        with self.assertRaises(ValueError):self.core.command(self.request())

    def test_busy_axis_and_disconnected_rejected(self):
        self.core.link.actual['axis_state']=3
        with self.assertRaises(ValueError):self.core.command(self.request())
        self.core.link.actual['axis_state']=6;self.core.link.bound=False
        with self.assertRaises(ValueError):self.core.command(self.request())

    def test_storage_failure_keeps_visible_record(self):
        with patch('robot.workbench.storage.write_json',side_effect=OSError('disk full')):
            with self.assertRaises(OSError):self.core.command(self.request())
        self.assertEqual(self.core.sequence_archive,[])
        self.assertEqual(len(self.core.visible_sequences()),2)

    def test_restart_keeps_archive_and_rejects_tampering(self):
        self.core.command(self.request())
        self.core.sequence_path.write_text(json.dumps(self.core.sequences),encoding='utf8')
        loaded=DeviceCore(runtime=self.folder.name)
        self.assertEqual(loaded.visible_sequences(),[self.second])
        data=json.loads(self.core.sequence_archive_path.read_text());data[0]['record']['rows'][0]['rpm']=99
        self.core.sequence_archive_path.write_text(json.dumps(data))
        with self.assertRaises(ValueError):DeviceCore(runtime=self.folder.name)

    def test_deleted_cannot_run_and_new_revision_not_reused(self):
        self.core.command(self.request())
        self.core.command(self.request(request_id='delete2',args=dict(id='测试',revision=2,sha256=self.second['sha256'],confirmed=True)))
        token=self.core.command(dict(op='claim',client='test',request_id='claim'))['token']
        base=dict(op='mechanism',client='test',token=token)
        with self.assertRaises(ValueError):self.core.command(dict(base,request_id='run',args=dict(action='program_run',id='测试',revision=1)))
        result=self.core.command(dict(base,request_id='save',args=dict(action='program_save',id='测试',rows=sequence_fixture.SequenceTests().rows())))
        self.assertEqual(result['revision'],3)
        self.assertEqual([r['revision'] for r in self.core.visible_sequences()],[3])


if __name__=='__main__':unittest.main()
