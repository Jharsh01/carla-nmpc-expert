"""How far along the lanes beside the car's own it can see, from a forward depth camera: the room to judge an overtake
by. A car the world model has not reported may be anywhere the car cannot see, so the overtake's clear check puts an
oncoming car at the edge of sight (core/urban constraints).

A CARLA depth camera stands in for a depth sensor on the car (a stereo rig or a lidar), as the world model's other
privileged sensors do; it is installed by the agent, outside the leaderboard's sensor list, and never seen by
SimLingo. Each frame gives every pixel's distance along the camera's axis. The lane beside the car's is walked ahead
from the map, every SPACING metres, at TARGET_HEIGHT above the road (about the middle of a car's body); each point is
projected into the latest frame, and is seen if it is in the frame and nothing nearer stands in front of it there.
The lane is seen as far as the first point that is not.

All in CARLA's frame (left-handed: x forward, y right, z up), as the camera and the map are.
"""
import math
import os
import threading

import numpy as np

SPACING = 2.0  # [m] between the points walked along a lane
REACH = 200.0  # [m] the furthest a lane is walked
TARGET_HEIGHT = 1.0  # [m] above the road: a car there is seen if this much of it is
NEAR = 5.0  # [m] points nearer the camera than this, beside the car and below the frame, are taken as seen
CAMERA = {"x": 0.5, "z": 2.5, "width": 640, "height": 160, "fov": 100.0}  # at the world model's eye (actors.py)


def depth_of(bgra):
    """Metres along the camera's axis from a CARLA depth image's BGRA bytes (h, w, 4): (R + 256 G + 256^2 B) / (256^3 - 1)
    of 1000 m."""
    b, g, r = (bgra[..., i].astype(np.float64) for i in range(3))
    return 1000.0 * (r + 256.0 * g + 65536.0 * b) / (256.0 ** 3 - 1.0)


def world_to_camera_of(location, rotation, mount):
    """The world-to-camera matrix (4 x 4) of a camera mounted at `mount` (x, y, z in the car's frame, no rotation of its
    own) on a car at this CARLA location and rotation (degrees), in double precision. Not the image's own transform: on
    Town12 (a large map) it put the camera 1000 m and 5000 m off the car, though its rotation was right."""
    cy, sy = math.cos(math.radians(rotation.yaw)), math.sin(math.radians(rotation.yaw))
    cp, sp = math.cos(math.radians(rotation.pitch)), math.sin(math.radians(rotation.pitch))
    cr, sr = math.cos(math.radians(rotation.roll)), math.sin(math.radians(rotation.roll))
    rot = np.array([[cp * cy, cy * sp * sr - sy * cr, -cy * sp * cr - sy * sr],  # car to world, as carla.Transform
                    [cp * sy, sy * sp * sr + cy * cr, -sy * sp * cr + cy * sr],
                    [sp, -cp * sr, cp * cr]])
    position = np.array([location.x, location.y, location.z]) + rot @ np.asarray(mount, dtype=float)
    out = np.eye(4)
    out[:3, :3] = rot.T
    out[:3, 3] = -rot.T @ position
    return out


def project(points, world_to_camera, width, height, fov):
    """(u, v, forward distance) of world points (n, 3) in a camera of this size and horizontal field of view, given its
    world-to-camera matrix (4 x 4, carla.Transform.get_inverse_matrix)."""
    p = np.c_[points, np.ones(len(points))] @ np.asarray(world_to_camera).T
    forward, right, up = p[:, 0], p[:, 1], p[:, 2]
    f = 0.5 * width / math.tan(math.radians(0.5 * fov))
    with np.errstate(divide="ignore", invalid="ignore"):
        u = 0.5 * width + f * right / forward
        v = 0.5 * height - f * up / forward
    return u, v, forward


def seen_along(distances, points, depth, world_to_camera, fov):
    """How far along a lane (its points (n, 3) at `distances` from the car, ascending) is seen in `depth` (h, w), the
    frame taken from the camera at `world_to_camera`: the distance of the first point that is out of the frame or has
    something nearer in front of it, or the last point's if all are seen."""
    if len(points) == 0:
        return 0.0
    height, width = depth.shape
    u, v, forward = project(np.asarray(points, dtype=float), world_to_camera, width, height, fov)
    for d, ui, vi, fi in zip(distances, u, v, forward):
        if fi < NEAR:
            continue
        if not (0.0 <= ui < width and 0.0 <= vi < height):
            return float(d)
        # nearer than the point by more than the depth's own spread over a pixel and the road's camber
        if depth[int(vi), int(ui)] < fi - (1.0 + 0.02 * fi):
            return float(d)
    return float(distances[-1])


class Sight:
    """The camera on the car and the lanes beside the car's, walked on the map. `sides(frame_z)` gives how far the car
    sees along the lane on its left and on its right, and whether each runs against it (None: no lane there)."""

    def __init__(self, carla, world, carla_map, hero):
        self.carla, self.map, self.hero = carla, carla_map, hero
        bp = world.get_blueprint_library().find("sensor.camera.depth")
        for key in ("width", "height", "fov"):
            bp.set_attribute("image_size_x" if key == "width" else "image_size_y" if key == "height" else "fov",
                             str(CAMERA[key]))
        place = carla.Transform(carla.Location(x=CAMERA["x"], z=CAMERA["z"]))
        self.camera = world.spawn_actor(bp, place, attach_to=hero)
        self._lock = threading.Lock()
        self._latest = None  # the last depth frame (h, w), at most a tick old
        self.camera.listen(self._store)
        # SIGHT_DEBUG=<dir>: the frame, the lane's points and the camera of the first few plans, for looking into
        self.debug_dir, self.debug_left = os.environ.get("SIGHT_DEBUG"), 5

    def _store(self, image):
        bgra = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
        depth = depth_of(bgra)
        with self._lock:
            self._latest = depth

    def destroy(self):
        if self.camera is not None:
            self.camera.stop()
            self.camera.destroy()
            self.camera = None

    def _lane_points(self, left):
        """Points along the lane beside the car's on one side, ahead of it, and whether that lane runs against the car:
        the car's lane walked ahead, the neighbour taken at each step. ([], [], None) where there is none beside it."""
        tr = self.hero.get_transform()
        wp = self.map.get_waypoint(tr.location, project_to_road=True, lane_type=self.carla.LaneType.Driving)
        if wp is None:
            return [], [], None
        # a lane's left and right are its own: in a lane running against the car (out in the oncoming lane), the car's
        # left is that lane's right, and the car's way along it is backwards
        backwards = math.cos(math.radians(wp.transform.rotation.yaw - tr.rotation.yaw)) < 0.0
        distances, points, against = [], [], None
        cur, d = wp, 0.0
        while d <= REACH:
            side = cur.get_left_lane() if left != backwards else cur.get_right_lane()
            if side is None or side.lane_type not in (self.carla.LaneType.Driving, self.carla.LaneType.Bidirectional):
                break
            if against is None:  # against the car
                against = math.cos(math.radians(side.transform.rotation.yaw - tr.rotation.yaw)) < 0.0
            loc = side.transform.location
            distances.append(d)
            points.append((loc.x, loc.y, loc.z + TARGET_HEIGHT))
            nxt = cur.previous(SPACING) if backwards else cur.next(SPACING)
            if not nxt:
                break
            yaw = cur.transform.rotation.yaw
            cur = min(nxt, key=lambda w: abs((w.transform.rotation.yaw - yaw + 180.0) % 360.0 - 180.0))
            d += SPACING
        return distances, points, against

    def sides(self):
        """{"left": (seen [m], runs against the car) or None, "right": ...}: None where there is no lane on that side,
        or no frame yet."""
        with self._lock:
            latest = self._latest
        tr = self.hero.get_transform()
        world_to_camera = world_to_camera_of(tr.location, tr.rotation, (CAMERA["x"], 0.0, CAMERA["z"]))
        out = {}
        for name, left in (("left", True), ("right", False)):
            distances, points, against = self._lane_points(left)
            if latest is None or not points:
                out[name] = None
                continue
            depth = latest
            out[name] = (seen_along(distances, points, depth, world_to_camera, CAMERA["fov"]), bool(against))
            if self.debug_dir and self.debug_left > 0:
                os.makedirs(self.debug_dir, exist_ok=True)
                np.savez(os.path.join(self.debug_dir, f"sight_{self.debug_left}_{name}.npz"), depth=depth,
                         world_to_camera=world_to_camera, distances=distances, points=points, seen=out[name][0],
                         hero=np.array([self.hero.get_transform().location.x, self.hero.get_transform().location.y,
                                        self.hero.get_transform().location.z, self.hero.get_transform().rotation.yaw]))
        if self.debug_dir and self.debug_left > 0 and any(v is not None for v in out.values()):
            self.debug_left -= 1
        return out
