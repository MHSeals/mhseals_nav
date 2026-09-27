"""Allow only the rigid camera subtree, never remote localization TF."""

import math

from geometry_msgs.msg import TransformStamped


class CameraStaticTF:
    def __init__(self, root):
        if root.strip("/") in ("map", "odom", "base_link", "base_footprint", ""):
            raise ValueError("camera_root_frame must be a camera-only frame")
        self.root = root
        self.transforms = {}

    def update(self, packet):
        transforms = packet["transforms"]
        if len(transforms) > 256:
            raise ValueError("too many static transforms")
        for item in transforms:
            parent = item["header"]["frame_id"]
            child = item["child_frame_id"]
            if (
                any(
                    frame.strip("/")
                    in ("map", "odom", "base_link", "base_footprint", "")
                    for frame in (parent, child)
                )
                or child == self.root
            ):
                continue
            values = item["transform"]
            xyz = [float(values["translation"][key]) for key in ("x", "y", "z")]
            xyzw = [float(values["rotation"][key]) for key in ("x", "y", "z", "w")]
            if not all(map(math.isfinite, xyz + xyzw)) or not math.isclose(
                sum(v * v for v in xyzw), 1.0, abs_tol=0.01
            ):
                continue
            msg = TransformStamped()
            msg.header.frame_id, msg.child_frame_id = parent, child
            (
                msg.transform.translation.x,
                msg.transform.translation.y,
                msg.transform.translation.z,
            ) = xyz
            (
                msg.transform.rotation.x,
                msg.transform.rotation.y,
                msg.transform.rotation.z,
                msg.transform.rotation.w,
            ) = xyzw
            if len(self.transforms) < 256 or child in self.transforms:
                self.transforms[child] = msg
        accepted, parents = [], {self.root}
        for _ in range(len(self.transforms)):
            added = [
                msg
                for child, msg in self.transforms.items()
                if msg.header.frame_id in parents and child not in parents
            ]
            if not added:
                break
            accepted.extend(added)
            parents.update(msg.child_frame_id for msg in added)
        return accepted
