import csv
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace
from east_core import load_tester_config
from analysis.diagnose_east_timing import run
from test_execution_timing import SlowAxis

class HeadlessDiagnosticBoundary(unittest.TestCase):
    def test_default_and_sensor_recording_modes_never_open_physical_hardware(self):
        with tempfile.TemporaryDirectory() as folder, patch('analysis.diagnose_east_timing.open_real',side_effect=AssertionError('Hardware forbidden')):
            bridge=Path(folder)/'bridge.csv';rotary=Path(folder)/'rotary.csv'
            result=run(load_tester_config(),.2,.008,include_sensor=True,sensor_recording=bridge,rotary_recording=rotary)
            self.assertEqual(result['mode'],'SIMULATED / NO HARDWARE')
            self.assertGreater(result['sample_count'],0)
            self.assertEqual(result['error_count'],0)
            self.assertEqual(result['sensor_recording_errors'],[])
            self.assertGreater(result['rotary_recording_statistics']['delivered_samples'],0)
            with bridge.open(newline='') as handle:
                rows=list(csv.reader(handle))
            self.assertEqual(len(rows)-1,result['sample_count'])
            self.assertTrue(all(row[-1]=='SIMULATED' for row in rows[1:]))
            self.assertFalse(result['production_limits_changed'])

    def test_rejected_capture_reports_individual_property_latency(self):
        device=SimpleNamespace(axis0=SlowAxis(auxiliary=.02),system_stats=SimpleNamespace(uptime=1))
        with patch('analysis.diagnose_east_timing.MockDevice',return_value=device):
            result=run(load_tester_config(),.06,.008,max_capture_s=.005)
        self.assertEqual(result['sample_count'],0)
        self.assertGreater(result['error_count'],0)
        self.assertGreater(result['latency_s']['active_errors_s']['maximum'],.015)
        self.assertIsNotNone(result['errors'][0]['rejected_snapshot'])
