import unittest
from steering_diagnostics import Stream, spans, analyze


class DiagnosticsTests(unittest.TestCase):
    def test_alignment_does_not_use_future(self):
        stream = Stream([[100, 'a'], [200, 'b']])
        self.assertIsNone(stream.at(99))
        self.assertEqual(stream.at(150)[1], 'a')
        self.assertIsNone(stream.at(250, max_age=40))

    def test_false_frame_splits_episode(self):
        rows = [{'t': i*.01, 'dt': .01, 'on': v} for i,v in enumerate([True,False,True])]
        self.assertEqual(len(spans(rows,lambda r:r['on'])),2)

    def test_data_gap_splits_episode(self):
        rows = [{'t': t,'dt':.01} for t in [0,.01,.3]]
        self.assertEqual(len(spans(rows,lambda r:True)),2)

    def test_missing_data_is_not_zero(self):
        with self.assertRaises(ValueError):
            analyze({},'missing')

    def test_max_command_does_not_imply_warning_or_saturation(self):
        data = {k:[] for k in ['carState','carControl','carOutput','controlsState']}
        for i in range(401):
            t=i*10_000_000
            data['carState'].append([t,5,5,10,0,0,0,i==250,False,False,False,False])
            data['carControl'].append([t,False,True,1])
            data['carOutput'].append([t,1,0,0,300])
            data['controlsState'].append([t,.01,.02,'pidState',True,False,1,0,0,0,10,15,0])
        result = analyze(data,'synthetic')
        self.assertEqual(result['saturated_s'],0)
        self.assertEqual(result['warnings'],[])
        self.assertGreater(result['at_limit_active_s'],4)
        self.assertAlmostEqual(result['eligible_s'],.5,places=2)
        self.assertEqual(result['car_output_max_abs_can'],300)


if __name__=='__main__':
    unittest.main()
