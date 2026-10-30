"""Evidence promotion preserves capture bytes and rejects unidentifiable firmware."""
import hashlib
import json
from pathlib import Path
import sys
import types
import importlib
import tempfile
import unittest
from unittest.mock import patch
# Load the hardware-independent helpers without the package's serial-driver exports.
package = types.ModuleType('_evidence_host')
package.__path__ = [str(Path(__file__).resolve().parents[1]/'hil/tinyg_hil')]
sys.modules[package.__name__] = package
evidence = importlib.import_module('_evidence_host.evidence')


class EvidencePromotion(unittest.TestCase):
    def test_unknown_firmware_is_not_promoted(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'run.json').write_text('{"firmware":{"known":false}}')
            with self.assertRaisesRegex(ValueError,'known firmware'):
                evidence.promote(root,'E015','before',root/'store')
            self.assertFalse((root/'store').exists())

    def test_capture_identity_and_derived_data_are_distinct(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);run=root/'run';run.mkdir();stem='profile-X-0.01mm'
            firmware={'known':True,'dirty':False,'commit':'12345678'}
            (run/'run.json').write_text(json.dumps({'firmware':firmware,'commit':'abcdef12','identity':{'id':'bench'}}))
            capture=b'capture bytes preserved exactly';(run/f'{stem}.sal').write_bytes(capture)
            (run/f'{stem}-edges.json').write_text(json.dumps({'sample_rate':12000000,'step_rises':[0,.01,.02,.03]}))
            info={'length_mm':.01,'steps_per_mm':1280,'feed_mm_min':600,'commanded_jerk_mm_s3':100,'raw':{'duplicated':'trace'},'windows':[]}
            (run/f'{stem}-analysis.json').write_text(json.dumps(info))
            def render(edges,analysis,path):
                path.write_bytes(b'plot');return evidence.analyze(edges['step_rises'],1280,windows=())
            with patch.object(evidence,'_plot_module'),patch.object(evidence,'_plot_case',side_effect=render):
                dest,summary=evidence.promote(run,'E015','before',root/'store')
            self.assertEqual((dest/f'{stem}.sal').read_bytes(),capture)
            self.assertEqual(summary['firmware'],firmware)
            self.assertEqual(summary['harness_commit'],'abcdef12')
            self.assertEqual(summary['cases'][0]['peak_jerk_ratio'],None)
            compact=json.loads((dest/f'{stem}-analysis.json').read_text())
            self.assertNotIn('raw',compact)
            entry=next(x for x in summary['artifacts'] if x['file'].endswith('.sal'))
            self.assertEqual(entry['sha256'],hashlib.sha256(capture).hexdigest())
            with self.assertRaises(FileExistsError):evidence.promote(run,'E015','before',root/'store')


if __name__=='__main__':unittest.main()
