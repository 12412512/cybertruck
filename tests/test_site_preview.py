"""标记起点条件下复用正式C闭环，不代表实车场地验收。"""

import json
import os
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))
from robot.nav.site import read_site
from robot.nav.site_preview import simulate
from robot.web.app import create_app
from fastapi.testclient import TestClient


class SitePreviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        library = ROOT / 'output/tests' / ('site-preview.dll' if os.name == 'nt' else 'site-preview.so')
        subprocess.run([sys.executable, '-B', str(ROOT / 'tools/build_odom.py'),
                        '--output', str(library)], check=True, capture_output=True)
        cls.site = read_site()
        cls.report = simulate(library, cls.site)

    def test_conditioned_diagonal_completes_without_changing_measured_start(self):
        report = self.report
        self.assertEqual(report['state'], 'COMPLETE')
        self.assertLess(report['terminal_error_m'], .005)
        self.assertLessEqual(report['peak_rpm'], 6)
        self.assertEqual(report['collisions'], [])
        self.assertAlmostEqual(report['boundary_clearance_m'], .005)
        self.assertEqual(report['start'], [.15, 2.25, 0])
        self.assertEqual(self.site['start']['from_left_m'], .15)
        self.assertEqual(self.site['previous_start']['from_left_m'], .145)
        self.assertEqual(self.site, read_site())
        self.assertFalse(report['placement_confirmed'])
        self.assertFalse(report['executable'])
        self.assertFalse(report['firmware_compatible'])

    def test_web_rejects_report_after_configuration_changes(self):
        client = TestClient(create_app())
        with patch('robot.nav.site.read_site', return_value=self.site), \
                patch('robot.web.app.Path.read_text', return_value=json.dumps(self.report)):
            self.assertTrue(client.get('/api/site/start1-qr/preview').json()['available'])
        changed = dict(self.report, site_sha256='stale')
        with patch('robot.nav.site.read_site', return_value=self.site), \
                patch('robot.web.app.Path.read_text', return_value=json.dumps(changed)):
            self.assertFalse(client.get('/api/site/start1-qr/preview').json()['available'])
