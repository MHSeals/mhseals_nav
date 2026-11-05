"""Sensor frames from the robot description, without ROS: base_link <- frame as (R, t).

The URDF (description/<robot>/<robot>.xacro) is the source of the sensor mounts, but for a lidar
mount the vessel profile gives (``lidar_configuration.profile_mount``), which wins over the
description's lidar_* properties (``vessel_properties``). TF reads it through
robot_state_publisher (odom.launch.py expands it with ``description_xml``), and anything that
runs without TF (the ROS-free tests, the sims' adapters) reads it here. Real xacro expands the file when it is installed;
otherwise a minimal expander handles what the descriptions use for their joints: top-level
``xacro:property`` values and ``${expr}`` arithmetic over them.
"""
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np

from mhseals_nav.layers import package_path

XACRO = '{http://www.ros.org/wiki/xacro}'


def description_dir():
    """Return this package's description/ (layers.package_path: the source tree, else the share)."""
    return package_path('description')


def description_path(robot='blastoise'):
    return description_dir() / robot / f'{robot}.xacro'


def quat_matrix(x, y, z, w):
    """Rotation matrix of a unit quaternion (a TF transform's rotation)."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rpy_matrix(roll, pitch, yaw):
    """URDF fixed-axis roll, pitch, yaw: Rz(yaw) Ry(pitch) Rx(roll), broadcast to (..., 3, 3)."""
    roll, pitch, yaw = np.broadcast_arrays(*(np.asarray(a, float) for a in (roll, pitch, yaw)))
    cr, sr, cp, sp, cy, sy = (f(a) for a in (roll, pitch, yaw) for f in (np.cos, np.sin))
    R = np.empty(roll.shape + (3, 3))
    R[..., 0, :] = np.stack([cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], -1)
    R[..., 1, :] = np.stack([sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], -1)
    R[..., 2, :] = np.stack([-sp, cp * sr, cp * cr], -1)
    return R


def matrix_rpy(R):
    """Rotation matrix -> (roll, pitch, yaw), the inverse of rpy_matrix."""
    return (math.atan2(R[2][1], R[2][2]), -math.asin(max(-1.0, min(1.0, R[2][0]))),
            math.atan2(R[1][0], R[0][0]))


def yaw(q):
    """Heading of a quaternion with x, y, z, w attributes (a TF rotation, a pose orientation)."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def matrix_quaternion(R):
    """Rotation matrix -> (x, y, z, w), w >= 0."""
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return x, y, z, w


def vessel_properties(text):
    """The vessel profile's overrides of this description's properties ({} without any).

    A description without the lidar_* properties (another robot's) keeps its own mount.
    """
    from .lidar_configuration import profile_mount
    found = profile_mount()
    return {name: value for name, value in found.items()
            if re.search(rf'<xacro:property name="{re.escape(name)}" value=', text)}


def description_xml(path, properties=None):
    """The URDF XML of a .xacro description with the vessel's overrides, then ``properties``."""
    import xacro
    text = Path(path).read_text()
    overrides = {**vessel_properties(text), **(properties or {})}
    if not overrides:
        return xacro.process_file(str(path)).toxml()
    return ET.tostring(expand(text, overrides), encoding='unicode')


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
        """Base <- frame tree of ``path``: the vessel's overrides, then ``properties``."""
        text = (Path(path) if path else description_path()).read_text()
        return cls(expand(text, {**vessel_properties(text), **(properties or {})}), base)

    @classmethod
    def from_transforms(cls, transforms, base='base_link'):
        """Build the tree of static TF transforms [(parent, child, R, t)], e.g. /tf_static."""
        geometry = cls.__new__(cls)
        geometry.base = base
        geometry.joints = {child: (parent, 'fixed', np.asarray(R, float), np.asarray(t, float))
                           for parent, child, R, t in transforms}
        return geometry

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


def fcu_lever_arms(geometry=None):
    """Return ArduPilot lever arms (FRD metres from base_link, to the mm) from the URDF."""
    geometry = geometry or RobotGeometry.load()
    out = {}
    for prefixes, frame in [(['GPS1_POS'], 'gps_link'),
                            (['INS_POS1', 'INS_POS2', 'INS_POS3'], 'imu_link')]:
        x, y, z = geometry.pose(frame)[1]
        for prefix in prefixes:
            out.update({f'{prefix}_X': round(x, 3) + 0.0, f'{prefix}_Y': round(-y, 3) + 0.0,
                        f'{prefix}_Z': round(-z, 3) + 0.0})
    return out


if __name__ == '__main__':
    print('\n'.join(f'{name},{value:.3f}' for name, value in fcu_lever_arms().items()))
