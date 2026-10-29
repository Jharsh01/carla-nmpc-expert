"""Conversions between the library's frame and CARLA's.

The NMPC (core/urban) works in a right-handed world frame: x forward, y to the left, heading psi counter-clockwise, angles in radians, at
the centre of gravity. CARLA (Unreal Engine) is left-handed: y points to the
right, yaw turns clockwise, angles are in degrees, and an actor's location is
its origin, not its centre of gravity. OpenDRIVE is right-handed; CARLA flips y
when it loads a map, so an OpenDRIVE point (x, y) is CARLA's (x, -y), and the
library's coordinates can be written into OpenDRIVE as they are.

The functions here take and return plain numbers so they can be tested without
CARLA; the agents wrap them around CARLA's types.
"""
import math


def to_carla_xy(x, y):
    return x, -y


def from_carla_xy(x, y):
    return x, -y


def to_carla_yaw(psi):
    """Heading [rad, counter-clockwise] to CARLA yaw [deg, clockwise]."""
    return -math.degrees(psi)


def from_carla_yaw(yaw_deg):
    return -math.radians(yaw_deg)


def origin_from_cg(x, y, psi, cg_ahead):
    """Actor origin of a car whose centre of gravity is at (x, y), `cg_ahead` metres in front of the origin."""
    return x - cg_ahead * math.cos(psi), y - cg_ahead * math.sin(psi)


def state_from_carla(location_xy, yaw_deg, velocity_xy, yaw_rate_deg, cg_ahead, delta):
    """The library's state [X, Y, psi, Ux, Uy, r, delta] from CARLA readings of a vehicle.

    `location_xy` and `yaw_deg` are the actor's transform, `velocity_xy` its
    velocity and `yaw_rate_deg` the z component of its angular velocity [deg/s],
    all in CARLA's frame; CARLA reports the velocity of the actor origin. The
    steering angle is not read from CARLA: `delta` is the angle the bridge
    commanded, as a steering-angle sensor would report it.
    """
    X0, Y0 = from_carla_xy(*location_xy)
    psi = from_carla_yaw(yaw_deg)
    vx, vy = from_carla_xy(*velocity_xy)
    r = -math.radians(yaw_rate_deg)
    c, s = math.cos(psi), math.sin(psi)
    Ux = c * vx + s * vy
    Uy_origin = -s * vx + c * vy
    # the centre of gravity is ahead of the origin, so it moves sideways faster by r * cg_ahead
    return [X0 + cg_ahead * c, Y0 + cg_ahead * s, psi, Ux, Uy_origin + r * cg_ahead, r, delta]


def velocity_to_carla(psi, Ux, Uy=0.0):
    """World velocity in CARLA's frame of a car with heading psi and body velocities Ux, Uy."""
    c, s = math.cos(psi), math.sin(psi)
    return to_carla_xy(c * Ux - s * Uy, s * Ux + c * Uy)


class Frame:
    """The car's own frame (its origin, x forward, y left), placed in the library's world frame: for turning what is
    known in the world (the world model's ground truth) into what a sensor on the car would report, and back."""

    def __init__(self, x, y, psi):
        self.x, self.y, self.psi = x, y, psi
        self.c, self.s = math.cos(psi), math.sin(psi)

    @classmethod
    def of(cls, transform):
        """The frame of an actor with this CARLA transform, at its origin."""
        return cls(*from_carla_xy(transform.location.x, transform.location.y), from_carla_yaw(transform.rotation.yaw))

    def to_car(self, x, y):
        dx, dy = x - self.x, y - self.y
        return self.c * dx + self.s * dy, -self.s * dx + self.c * dy

    def to_world(self, x, y):
        return self.x + self.c * x - self.s * y, self.y + self.s * x + self.c * y
