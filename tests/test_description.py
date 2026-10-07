"""Check the rendered robot geometry agrees with the independent TCP model."""
import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/a2_demo'))
from a2_demo.core import Cycle, inverse


class DescriptionTests(unittest.TestCase):
    def setUp(self):
        self.robot = ET.parse(ROOT/'src/a2_demo/urdf/scara.urdf').getroot()

    def test_urdf_chain_tcp_matches_every_target(self):
        joints = {j.find('child').attrib['link']: j for j in self.robot.findall('joint')}
        chain = []
        child = 'tool0'
        while child != 'world':
            joint = joints[child]
            chain.insert(0, joint)
            child = joint.find('parent').attrib['link']
        for step in Cycle().plan:
            if step.kind != 'arm':
                continue
            q = dict(zip(('shoulder_joint', 'elbow_joint', 'slide_joint', 'wrist_joint'), inverse(step.target)))
            x = y = z = yaw = 0.0
            for joint in chain:
                origin = joint.find('origin')
                dx, dy, dz = [float(v) for v in origin.attrib.get('xyz', '0 0 0').split()] if origin is not None else (0, 0, 0)
                x += math.cos(yaw)*dx-math.sin(yaw)*dy
                y += math.sin(yaw)*dx+math.cos(yaw)*dy
                z += dz
                kind = joint.attrib['type']
                if kind == 'revolute':
                    yaw += q[joint.attrib['name']]
                elif kind == 'prismatic':
                    axis = [float(v) for v in joint.find('axis').attrib['xyz'].split()]
                    z += axis[2]*q[joint.attrib['name']]
            self.assertLess(math.dist((x, y, z), step.target), 1e-9)
            self.assertAlmostEqual(yaw, 0.0)

    def test_all_targets_within_urdf_joint_limits(self):
        names = ('shoulder_joint', 'elbow_joint', 'slide_joint', 'wrist_joint')
        limits = {j.attrib['name']: j.find('limit').attrib for j in self.robot.findall('joint') if j.find('limit') is not None}
        for step in Cycle().plan:
            if step.kind == 'arm':
                for name, value in zip(names, inverse(step.target)):
                    self.assertGreaterEqual(value, float(limits[name]['lower']))
                    self.assertLessEqual(value, float(limits[name]['upper']))

    def test_urdf_links_are_unique_and_connected(self):
        links = [link.attrib['name'] for link in self.robot.findall('link')]
        self.assertEqual(len(links), len(set(links)))
        children = []
        for joint in self.robot.findall('joint'):
            self.assertIn(joint.find('parent').attrib['link'], links)
            child = joint.find('child').attrib['link']
            self.assertIn(child, links)
            children.append(child)
        self.assertEqual(set(children), set(links)-{'world'})
        self.assertEqual(len(children), len(set(children)))


if __name__ == '__main__':
    unittest.main()
