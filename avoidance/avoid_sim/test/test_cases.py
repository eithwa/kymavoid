#!/usr/bin/env python3
"""avoid_sim.cases: recorded case files round-trip, keep their header, refuse bad cases."""
import os
import tempfile
import unittest

from avoid_sim import cases


class CaseFileTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.path = os.path.join(self.dir, 'sub', 'recorded.yaml')

    def make(self, **kw):
        return cases.make(7, 'random', (-2.0, 0.0, 0.0174533), [(0.1, 0.2), (1.0, -0.5)], 'camera', **kw)

    def test_round_trip(self):
        cid = cases.append(self.path, self.make(note='路線很寬卻很慢', expect={'result': 'success', 'max_time': 7.5},
                                                 case_id='wide-gap'))
        self.assertEqual(cid, 'wide-gap')
        c = cases.load(self.path)[0]
        self.assertEqual(c['seed'], 7)
        self.assertEqual(c['start'], [-2.0, 0.0, 1.0])                  # yaw stored in degrees
        self.assertEqual(c['obstacles'], [[0.1, 0.2], [1.0, -0.5]])
        self.assertEqual(c['note'], '路線很寬卻很慢')                      # unicode kept readable
        self.assertEqual(c['expect'], {'result': 'success', 'max_time': 7.5})
        self.assertEqual(c['status'], 'open')
        with open(self.path) as fh:
            self.assertIn('路線很寬卻很慢', fh.read())

    def test_unique_ids_and_header_kept(self):
        cases.append(self.path, self.make(case_id='a'))
        with open(self.path) as fh:
            text = fh.read()
        with open(self.path, 'w') as fh:
            fh.write('# my own header\n' + text[text.index('cases:'):])
        self.assertEqual(cases.append(self.path, self.make(case_id='a')), 'a-2')
        self.assertEqual(cases.append(self.path, self.make(case_id='a')), 'a-3')
        with open(self.path) as fh:
            self.assertTrue(fh.read().startswith('# my own header\n'))
        self.assertEqual([c['id'] for c in cases.load(self.path)], ['a', 'a-2', 'a-3'])

    def test_default_id_and_expect(self):
        c = self.make()
        self.assertTrue(c['id'].startswith('case-') and c['id'].endswith('-seed7'))
        self.assertEqual(c['expect'], {'result': 'success'})

    def test_rejects_bad_cases(self):
        with self.assertRaises(ValueError):
            cases.append(self.path, self.make(expect={'max_speed': 1.0}))
        with self.assertRaises(ValueError):
            cases.append(self.path, self.make(status='maybe'))
        bad = self.make()
        del bad['obstacles']
        with self.assertRaises(ValueError):
            cases.validate(bad)
        self.assertFalse(os.path.exists(self.path))                         # nothing written

    def test_resolve(self):
        self.assertEqual(os.path.basename(cases.resolve('regression')), 'regression.yaml')
        self.assertEqual(cases.resolve(self.path), self.path)

    def test_shipped_case_files_are_valid(self):
        d = cases.cases_dir()
        files = [f for f in os.listdir(d) if f.endswith('.yaml')]
        self.assertTrue(files)
        for f in files:
            self.assertTrue(cases.load(os.path.join(d, f)))


if __name__ == '__main__':
    unittest.main()
