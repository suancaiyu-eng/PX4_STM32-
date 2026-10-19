import importlib.util
from pathlib import Path
import re
import struct
import unittest
import zlib
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('calibrate_mag',ROOT/'Tools/calibrate_mag.py')
cal=importlib.util.module_from_spec(spec); spec.loader.exec_module(cal)


class CalibrationTests(unittest.TestCase):
    def cloud(self):
        rng=np.random.default_rng(303)
        directions=rng.normal(size=(3000,3)); directions/=np.linalg.norm(directions,axis=1)[:,None]
        truth=np.array([[1.1,.03,-.02],[.03,.9,.01],[-.02,.01,1.04]])
        bias=np.array([-35.,-34.,17.])
        points=(directions*50)@np.linalg.inv(truth).T+bias+rng.normal(scale=.04,size=(3000,3))
        return points,bias,truth

    def test_recovers_known_ellipsoid_and_holdout(self):
        points,bias,truth=self.cloud(); actual,matrix,report=cal.fit(points)
        np.testing.assert_allclose(actual,bias,atol=.05)
        np.testing.assert_allclose(matrix,truth,atol=.002)
        self.assertLess(report['holdout_rms_relative'],.003)
        self.assertEqual(len(report['octants']),8)

    def test_rejects_stationary_planar_and_incomplete_coverage(self):
        points,_,_=self.cloud()
        for bad in [np.ones((1000,3)),np.column_stack([points[:1000,:2],np.zeros(1000)]),
                    points[:100],points[points[:,2]>17]]:
            with self.subTest(shape=bad.shape),self.assertRaises(ValueError): cal.fit(bad)

    def test_rejects_inconsistent_magnetic_environment(self):
        points,_,_=self.cloud(); points[::2]+=np.array([20,-15,10])
        with self.assertRaises(ValueError): cal.fit(points)

    def test_generated_header_crc_matches_its_float32_constants(self):
        points,_,_=self.cloud(); bias,matrix,_=cal.fit(points); text=cal.header(bias,matrix)
        all_values=[]
        for macro in ['BIAS','MATRIX']:
            line=next(line for line in text.splitlines() if line.startswith('#define MAG_CALIBRATION_'+macro+' '))
            all_values.extend(float(value) for value in re.findall(r'(-?[\d.]+(?:e[+-]?\d+)?)f',line))
        all_values.append(50.)
        expected=int(re.search(r'CRC32 0x([0-9A-F]+)',text)[1],16)
        self.assertEqual(zlib.crc32(struct.pack('<13f',*all_values)),expected)


if __name__=='__main__': unittest.main()
