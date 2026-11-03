"""Check the Gazebo robot actually comes from the upstream Spot description."""
from pathlib import Path
from urllib.parse import urlparse, unquote
import math
import unittest
import xml.etree.ElementTree as E
from spot_gazebo.model import JOINT_NAMES, STAND_HIP, STAND_KNEE, SIT_HIP, SIT_KNEE

ROOT = Path(__file__).resolve().parents[1]


class ModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.urdf = E.parse(ROOT/'models/spot/spot.urdf').getroot()
        cls.model = E.parse(ROOT/'worlds/spot.sdf').find('world/model[@name="spot"]')

    def test_all_12_upstream_joints_and_limits_preserved(self):
        source = {j.attrib['name']: j for j in self.urdf.findall("joint[@type='revolute']")}
        self.assertEqual(set(source), set(JOINT_NAMES.values()))
        for name, joint in source.items():
            imported = self.model.find(f"joint[@name='{name}']")
            self.assertIsNotNone(imported, name)
            for field in ('lower', 'upper', 'effort', 'velocity'):
                self.assertAlmostEqual(float(joint.find('limit').attrib[field]),
                                       float(imported.findtext(f'axis/limit/{field}')), msg=name)
            self.assertEqual(joint.find('axis').attrib['xyz'], imported.findtext('axis/xyz'))
            expected = list(map(float, joint.find('origin').attrib['xyz'].split()))
            actual = list(map(float, imported.findtext('pose').split()))[:3]
            for a, b in zip(expected, actual):
                self.assertAlmostEqual(a, b, msg=name)

    def test_visuals_use_upstream_meshes_not_primitives(self):
        visuals = self.model.findall('link/visual')
        self.assertEqual(len(visuals), 13)
        for visual in visuals:
            self.assertIsNotNone(visual.find('geometry/mesh'))
            self.assertIsNone(visual.find('geometry/box'))
            asset = Path(unquote(urlparse(visual.findtext('geometry/mesh/uri')).path))
            self.assertTrue(asset.is_file(), asset)
            self.assertIn('/meshes/base/visual/', str(asset))
            for line in asset.read_text().splitlines():
                if line.startswith('mtllib '):
                    self.assertTrue((asset.parent/line[7:].strip()).is_file())

    def test_collision_meshes_available(self):
        meshes = self.model.findall('link/collision/geometry/mesh')
        self.assertEqual(len(meshes), 9)
        for mesh in meshes:
            self.assertTrue(Path(unquote(urlparse(mesh.findtext('uri')).path)).is_file())

    def test_nominal_targets_fit_upstream_joint_limits(self):
        for alias, name in JOINT_NAMES.items():
            limit = self.model.find(f"joint[@name='{name}']/axis/limit")
            low, high = float(limit.findtext('lower')), float(limit.findtext('upper'))
            values = ([STAND_HIP-.22, STAND_HIP+.22, SIT_HIP] if alias.endswith('_hy') else
                      [STAND_KNEE-.25, STAND_KNEE, SIT_KNEE] if alias.endswith('_kn') else [0.])
            for value in values:
                self.assertTrue(math.isfinite(value) and low <= value <= high, (name, value))
