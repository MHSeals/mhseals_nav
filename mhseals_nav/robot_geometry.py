"""Sensor frames from the robot description, without ROS: base_link <- frame as (R, t).

The URDF (description/<robot>/<robot>.xacro) is the one source of the sensor mounts; TF
reads it through robot_state_publisher, and anything that runs without TF (the ROS-free
tests, the sims' adapters) reads it here. Real xacro expands the file when it is installed;
otherwise a minimal expander handles what the descriptions use for their joints: top-level
``xacro:property`` values and ``${expr}`` arithmetic over them.
"""
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np

DESCRIPTION = Path(__file__).resolve().parents[1] / 'description'
XACRO = '{http://www.ros.org/wiki/xacro}'


def description_path(robot='blastoise'):
    return DESCRIPTION / robot / f'{robot}.xacro'


def rpy_matrix(roll, pitch, yaw):
    """URDF fixed-axis roll, pitch, yaw: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def matrix_quaternion(R):
    """Rotation matrix -> (x, y, z, w), w >= 0."""
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return x, y, z, w


def expand(text, properties=None):
    """Expand xacro text into a URDF root; ``properties`` override the file's values."""
    try:
        import xacro
    except ImportError:
        return _expand_lite(text, properties or {})
    import tempfile
    with tempfile.NamedTemporaryFile('w', suffix='.xacro') as f:
        f.write(_override(text, properties or {}))
        f.flush()
        return ET.fromstring(xacro.process_file(f.name).toxml())


def _override(text, properties):
    for name, value in properties.items():
        text, n = re.subn(rf'(<xacro:property name="{re.escape(name)}" value=")[^"]*"',
                          rf'\g<1>{value}"', text)
        if n != 1:
            raise KeyError(f'xacro property {name!r} is not defined once')
    return text


def _expand_lite(text, properties):
    root = ET.fromstring(_override(text, properties))
    scope = {'pi': math.pi}

    def subst(raw):
        return re.sub(r'\$\{([^}]*)\}',
                      lambda m: str(eval(m.group(1), {'__builtins__': {}}, scope)), raw)

    for prop in root.findall(f'{XACRO}property'):
        raw = subst(prop.get('value'))
        try:
            scope[prop.get('name')] = float(raw)
        except ValueError:
            scope[prop.get('name')] = raw
    for node in root.iter():
        for key, raw in node.attrib.items():
            node.set(key, subst(raw))
    return root


class RobotGeometry:
    """Fixed-joint tree of a robot description; ``pose(frame)`` is base <- frame."""

    def __init__(self, root, base='base_link'):
        self.base = base
        self.joints = {}
        for joint in root.findall('joint'):
            child = joint.find('child').get('link')
            if child in self.joints:
                raise ValueError(f'{child} has two parents')
            origin = joint.find('origin')
            xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None
                                      else '0 0 0').split()]
            rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None
                                      else '0 0 0').split()]
            self.joints[child] = (joint.find('parent').get('link'), joint.get('type'),
                                  rpy_matrix(*rpy), np.array(xyz))

    @classmethod
    def load(cls, path=None, properties=None, base='base_link'):
        path = Path(path) if path else description_path()
        return cls(expand(path.read_text(), properties), base)

    def chain(self, frame):
        links = [frame]
        while links[-1] != self.base:
            if links[-1] not in self.joints:
                raise KeyError(f'{links[-1]} does not reach {self.base}')
            links.append(self.joints[links[-1]][0])
        return links

    def pose(self, frame):
        R, t = np.eye(3), np.zeros(3)
        for link in reversed(self.chain(frame)[:-1]):
            _, kind, Rj, tj = self.joints[link]
            if kind != 'fixed':
                raise ValueError(f'{link} hangs on a {kind} joint')
            R, t = R @ Rj, R @ tj + t
        return R, t

    def matrix(self, frame):
        R, t = self.pose(frame)
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, t
        return T
