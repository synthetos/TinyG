import importlib.util
from pathlib import Path
import sys
import unittest
ROOT=Path(__file__).resolve().parents[2]/'tools/perf'
sys.path.insert(0,str(ROOT))
import bench_compare as b


def row(tag,slot=1,seconds=10,**kw):
    return dict(kind='bench-perf',schema=1,fixture='05.nc',tag=tag,slot=str(slot),rt='off',bench='bench-3',
        source='mso',sample_rate_hz=2_000_000,identity={'id':'board','fb':449.01 if tag=='A' else 450.04},
        expected_fb=449.01 if tag=='A' else 450.04,expected_hex=None,
        flash={'known':True,'commit':tag*40,'hex_sha256':('a' if tag=='A' else 'b')*64},sha256='abc',settings={'values':{'xfr':800}},valid=True,
        fixed_snapshot={'sha256':'f'*64},fixed_verified={'ok':True},missing_motors=[],parse_bound=False,metrics={'motion_s':seconds,'motors':{
        f'M{i}':dict(total=10,net=0,reversal_legs=2,min_pulse_s=.000002,max_rate_hz=100) for i in range(1,5)}},
        serial={'completion':'idle','errors':[],'probe_mode':'off','rx_credit_initial':8,'rx_credit_min':8,'rx_credit_last':8,'lines_s':100,'stat_median_s':.01,'stat_p95_s':.02},**kw)


class BenchCompare(unittest.TestCase):
    def records(self): return [row(tag,i) for i in range(5) for tag in ('A','B')]
    def test_equal_passes_and_insufficient_does_not(self):
        self.assertEqual(b.compare(self.records())[0]['result'],'PASS')
        self.assertNotEqual(b.compare([row('A'),row('B')])[0]['result'],'PASS')
    def test_missing_or_duplicate_or_settings(self):
        for case in ('missing','duplicate','settings','motor','invalid'):
            rs=self.records()
            if case=='missing':rs.pop()
            if case=='duplicate':rs.append(rs[0])
            if case=='settings':rs[1]['settings']['values']['xfr']=900
            if case=='motor':rs[1]['metrics']['motors'].pop('M4')
            if case=='invalid':rs[1]['valid']=False
            with self.subTest(case=case): self.assertNotEqual(b.compare(rs)[0]['result'],'PASS')
    def test_thresholds_and_expected_note_never_hide_failure(self):
        for metric,value in [('motion_s',10.2),('lines_s',97)]:
            rs=self.records()
            for r in rs:
                if metric=='lines_s':r['parse_bound']=True;r['serial']['probe_mode']='off'
                if r['tag']=='B':r['metrics' if metric=='motion_s' else 'serial'][metric]=value
            self.assertEqual(b.compare(rs,{'05.nc':'differs:E180'})[0]['result'],'FAIL')
    def test_steps_and_pulse(self):
        for field,value in [('net',2),('reversal_legs',3),('min_pulse_s',.000001)]:
            rs=self.records();rs[1]['metrics']['motors']['M1'][field]=value
            self.assertEqual(b.compare(rs)[0]['result'],'FAIL')
    def test_speedup_is_review_and_rt_pairs_are_distinct(self):
        rs=self.records()
        for r in rs:
            if r['tag']=='B':r['metrics']['motion_s']=2
        self.assertEqual(b.compare(rs)[0]['result'],'FASTER-REVIEW')
        for r in rs:
            r['rt']='on' if r['tag']=='B' else 'off';r['tag']='B';r['metrics']['motion_s']=10
            r['identity']['fb']=r['expected_fb']=450.04;r['flash']={'known':True,'commit':'B'*40,'hex_sha256':'b'*64}
        self.assertEqual(b.compare(rs,rt_cost=True)[0]['result'],'PASS')
    def test_nonfinite_metrics_are_findings(self):
        rs=self.records();rs[1]['metrics']['motion_s']=float('nan')
        self.assertNotEqual(b.compare(rs)[0]['result'],'PASS')

    def test_firmware_provenance_cannot_be_mislabeled_or_mixed(self):
        for case in ('wrong_A','wrong_B','unknown','missing','expected_hash','mixed_hex','same_hex'):
            rs=self.records()
            if case=='wrong_A':rs[0]['identity']['fb']=450.04
            if case=='wrong_B':rs[1]['identity']['fb']=450.03
            if case=='unknown':rs[1]['flash']['known']=False
            if case=='missing':rs[1].pop('flash')
            if case=='expected_hash':rs[1]['expected_hex']='c'*64
            if case=='mixed_hex':rs[3]['flash']['hex_sha256']='c'*64
            if case=='same_hex':
                for r in rs:r['flash']['hex_sha256']='a'*64
            with self.subTest(case=case):self.assertNotEqual(b.compare(rs)[0]['result'],'PASS')

    def test_shared_settings_drift_and_missing_tokens(self):
        for case in ('bjd','cjd','missing','extra_unknown','sr'):
            rs=self.records()
            for r in rs:r['settings']['values'].update(bjd=.01,cjd=.01)
            if case in ('bjd','cjd'):rs[0]['settings']['values'][case]=0
            if case=='missing':rs[1]['settings']['values'].pop('bjd')
            if case=='extra_unknown':rs[1]['settings']['values']['mystery']=1
            if case=='sr':rs[1]['settings']['sr']=['stat','posx']
            with self.subTest(case=case):self.assertNotEqual(b.compare(rs)[0]['result'],'PASS')
        rs=self.records()
        for r in rs:
            if r['tag']=='B':r['settings']['values'].update(spph=0,spde=1,coph=0,rtfi=0)
        self.assertEqual(b.compare(rs)[0]['result'],'PASS')

    def test_fixed_snapshot_must_be_identical_and_verified(self):
        for case in ('different','missing','unverified'):
            rs=self.records()
            if case=='different':rs[1]['fixed_snapshot']['sha256']='e'*64
            if case=='missing':rs[1].pop('fixed_snapshot')
            if case=='unverified':rs[1]['fixed_verified']['ok']=False
            with self.subTest(case=case):self.assertNotEqual(b.compare(rs)[0]['result'],'PASS')

    def test_user_words_are_explicitly_ignored_but_motion_settings_are_not(self):
        rs=self.records()
        for r in rs:r['settings']['values']['udb2']='0x1' if r['tag']=='A' else '0x0'
        result=b.compare(rs)[0]
        self.assertEqual(result['result'],'PASS')
        self.assertIn('udb2',result['not_restorable'])
        rs[1]['settings']['values']['bjd']=0
        self.assertNotEqual(b.compare(rs)[0]['result'],'PASS')

    def test_only_documented_legacy_precision_can_explain_pair_difference(self):
        rs=self.records()
        for r in rs:
            r['settings']['values']['bjd']=0 if r['tag']=='A' else .05
            if r['tag']=='A':r['fixed_verified']['reduced_precision']=[dict(token='bjd',precision=0,expected=.05,actual=0,printed_expected=0)]
        self.assertEqual(b.compare(rs)[0]['result'],'PASS')
        self.assertIn('bjd',b.compare(rs)[0]['reduced_precision'])
        rs[0]['fixed_verified']['reduced_precision'][0]['token']='xfr'
        self.assertNotEqual(b.compare(rs)[0]['result'],'PASS')

    def test_precision_is_extracted_from_cfgarray_column(self):
        from extract_legacy_precision import extract
        source='\n{ "b", "bjd", _fip, 0, get_flt },\n{ "c", "cjd", _fip, 2, get_flt },\n'
        entries=extract(source)
        self.assertEqual(entries['bjd']['precision'],0)
        self.assertEqual(entries['cjd']['precision'],2)
        self.assertEqual(entries['cjd']['line'],3)

    def test_parse_bound_requires_probe_free_pairs(self):
        rs=self.records()
        for r in rs:
            r['parse_bound']=True
            r['serial'].update(probe_mode='off',stat_median_s=None,stat_p95_s=None)
        result=b.compare(rs)[0]
        self.assertEqual(result['result'],'PASS')
        self.assertNotIn('stat_median_s',result['ratios'])
        rs[0]['serial']['probe_mode']='250ms'
        self.assertNotEqual(b.compare(rs)[0]['result'],'PASS')

    def test_probe_credit_exhaustion_is_n55_finding_even_when_invalid(self):
        rs=self.records()
        for r in rs:r['parse_bound']=False;r['serial']['probe_mode']='250ms'
        rs[0]['serial']['rx_credit_min']=2
        rs[0]['valid']=False
        result=b.compare(rs)[0]
        self.assertTrue(any('N55' in s for s in result['findings']))
        self.assertNotEqual(result['result'],'PASS')

    def test_reduced_final_probe_window_is_finding_above_reserve(self):
        rs=self.records()
        rs[0]['serial'].update(probe_mode='250ms',rx_credit_initial=10,rx_credit_min=4,rx_credit_last=4)
        self.assertTrue(any('N55' in x for x in b.compare(rs)[0]['findings']))

    def test_absolute_b_round_trip_has_no_a_pair(self):
        r=row('B');r['perf_probes']='on';r['serial']['probe_mode']='250ms'
        result=b.round_trips([row('A'),r])[0]
        self.assertEqual(result['stat_median_s'],.01)
        self.assertEqual(result['result'],'MEASURED')
        self.assertIn('N55',result['baseline'])
        r['valid']=False
        self.assertEqual(b.round_trips([r])[0]['result'],'FINDING')

    def test_separate_b_probe_pass_does_not_pollute_pairs(self):
        rs=self.records();probe=row('B');probe['perf_probes']='on'
        probe['serial']['probe_mode']='250ms';rs.append(probe)
        self.assertEqual(b.compare(rs)[0]['result'],'PASS')

    def test_absolute_rtt_rejects_bad_evidence(self):
        for case in ('credits','settings','identity','timing'):
            r=row('B');r['perf_probes']='on';r['serial']['probe_mode']='250ms'
            if case=='credits':r['serial']['rx_credit_min']=2
            if case=='settings':r['fixed_verified']['ok']=False
            if case=='identity':r['identity']['fb']=449.01
            if case=='timing':r['serial']['stat_p95_s']=None
            with self.subTest(case=case):self.assertEqual(b.round_trips([r])[0]['result'],'FINDING')

    def test_multi_fixture_multi_slot_preserves_expectations(self):
        records=[]
        for fixture in ('01_long_straight_x.nc','05_tiny_collinear.nc','20_hold_resume.nc'):
            for slot in (1,2):
                for tag in ('A','B'):
                    r=row(tag,slot);r['fixture']=fixture
                    r['sha256']=fixture;r['perf_probes']='off'
                    records.append(r)
        for notes in (None,{'05_tiny_collinear.nc':'E180 review'}):
            result=b.compare(records,notes)
            self.assertEqual(len(result),3)
            self.assertTrue(all(r['pairs']==2 for r in result))
            self.assertEqual(result[1]['expect'],'same' if notes is None else 'E180 review')
        if notes:self.assertEqual(notes,{'05_tiny_collinear.nc':'E180 review'})

    def test_isolated_cli_imports_sibling_module(self):
        import json,subprocess,tempfile
        with tempfile.TemporaryDirectory(dir=ROOT) as folder:
            path=Path(folder)/'perf-input.json';path.write_text(json.dumps(row('A')))
            proc=subprocess.run([sys.executable,'-I',str(ROOT/'compare.py'),'--bench',str(path)],
                                capture_output=True,text=True)
        self.assertEqual(proc.returncode,1) # unpaired evidence is a finding, not a crash
        self.assertNotIn('Traceback',proc.stderr)
        self.assertIn('unpaired slot',proc.stdout)

    def test_pulse_comparison_uses_one_recorded_sample(self):
        for shortening,expected in ((.0000000008,'PASS'),(.0000005,'FAIL')):
            rs=self.records()
            for r in rs:
                r['sample_rate_hz']=2_000_000
                for m in r['metrics']['motors'].values():m['reversal_legs']=2
            rs[1]['metrics']['motors']['M1']['min_pulse_s']-=shortening
            self.assertEqual(b.compare(rs)[0]['result'],expected)

    def test_totals_need_equal_legs_and_net_not_equal_total(self):
        rs=self.records()
        for r in rs:
            r['sample_rate_hz']=2_000_000
            for m in r['metrics']['motors'].values():m['reversal_legs']=600
        rs[1]['metrics']['motors']['M1']['total']+=597
        result=b.compare(rs)[0]
        self.assertEqual(result['result'],'PASS')
        self.assertTrue(any('phase' in x for x in result['explanations']))
        rs[1]['metrics']['motors']['M1']['reversal_legs']=599
        self.assertEqual(b.compare(rs)[0]['result'],'FAIL')

    def test_speedup_attribution_is_explicit(self):
        for fixture,note in [('05_tiny_collinear.nc','E180'),('14_feed_changes.nc','E089'),('11_short_arcs.nc','unattributed')]:
            rs=self.records()
            for r in rs:
                r['fixture']=fixture;r['sample_rate_hz']=2_000_000
                for m in r['metrics']['motors'].values():m['reversal_legs']=2
                if r['tag']=='B':r['metrics']['motion_s']=9
            result=b.compare(rs)[0]
            self.assertEqual(result['result'],'FASTER-REVIEW')
            self.assertIn(note,result['attribution'])

    def test_old_capture_evidence_is_verified_before_use(self):
        import json,tempfile
        with tempfile.TemporaryDirectory(dir=ROOT) as folder:
            path=Path(folder)/'capture.json'
            path.write_text(json.dumps(dict(rate=2_000_000,initial={'s':0,'d':0},
                transitions={'s':[[1,1],[1.1,0],[2,1],[2.1,0]],'d':[[1.5,1]]})))
            def legacy():
                r=row('A');r.pop('sample_rate_hz');r['capture_file']=str(path)
                r['capture_channel_map']={'s':'M1 step','d':'M1 dir'}
                r['settings']['values']['1po']=0
                r['metrics']['motors']={'M1':dict(total=2,net=0)}
                return r
            r=legacy();b.capture_evidence(r)
            self.assertEqual(r['sample_rate_hz'],2_000_000)
            self.assertEqual(r['metrics']['motors']['M1']['reversal_legs'],2)
            r=legacy();r['metrics']['motors']['M1']['net']=1;b.capture_evidence(r)
            self.assertNotIn('sample_rate_hz',r)
            self.assertNotIn('reversal_legs',r['metrics']['motors']['M1'])

    def test_missing_rate_or_legs_stays_a_finding(self):
        for key in ('sample_rate_hz','reversal_legs'):
            rs=self.records()
            if key=='sample_rate_hz':rs[0].pop(key)
            else:rs[0]['metrics']['motors']['M1'].pop(key)
            self.assertEqual(b.compare(rs)[0]['result'],'FINDING')

    def test_speed_attribution_does_not_hide_a_net_failure(self):
        rs=self.records()
        for r in rs:
            r['fixture']='05_tiny_collinear.nc'
            if r['tag']=='B':r['metrics']['motion_s']=5
        rs[1]['metrics']['motors']['M1']['net']=2
        result=b.compare(rs)[0]
        self.assertEqual(result['result'],'FAIL')
        self.assertEqual(result['speed_review'],'FASTER-REVIEW')
        self.assertIn('E180',result['attribution'])
