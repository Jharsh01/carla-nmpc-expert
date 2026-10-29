# Vehicle parameters: CARLA's Lincoln MKZ 2020 (Bench2Drive's ego)

Bench2Drive always drives `vehicle.lincoln.mkz_2020` in CARLA 0.9.15, with its default physics. CARLA simulates it
with Unreal Engine 4.26's PhysX four-wheel vehicle. The parameters in `config/vehicles/mkz2020.json` describe that
car, and the urban NMPC plans with them; it is the only vehicle in the repository. The file holds only what the NMPC
uses. The rest of CARLA's physics settings are below, for reference.

**Sources**

- `Vehicle.get_physics_control()` on a CARLA 0.9.15 server, read on 29 October 2026. The raw readout is in
  [`reference/mkz2020_physics_control_0.9.15.json`](reference/mkz2020_physics_control_0.9.15.json), with wheel
  positions in the actor's frame.
- Driving tests by `scripts/identify_ego.py` (Town06, 20 Hz synchronous), whose tables are in
  `config/vehicles/mkz2020_actuation.json`. Where a test and the physics settings disagree, the test wins, since
  it measures what PhysX actually does.

## Read from CARLA

| Quantity | Value |
| --- | --- |
| Mass | 1696 kg |
| Drag coefficient | 0.30 |
| Centre of mass offset | (0.30, 0, −0.20) m from the actor origin |
| Wheel radius | 35.5 cm |
| Max steer angle, front wheels | 70° |
| Steering curve (speed → scale) | 0 → 1.0, 20 → 0.9, 60 → 0.8, 120 → 0.7 |
| Max brake torque, each wheel | 800 N·m (handbrake 1600 N·m, rear) |
| Tire friction, each wheel | 3.5 |
| Lateral stiffness value / max load | 20 / 3 |
| Longitudinal stiffness value | 3000 |
| Torque curve | 347 N·m at 1000 rpm, peak 695 N·m at 3000 rpm, 466 N·m at 6500 rpm (max) |
| Gears | 6 forward (4.584 … 0.746), final drive 3.21, automatic |
| Bounding box (L × W × H) | 4.89 × 1.84 × 1.49 m, centred 0.006 m behind the actor origin |
| Wheel positions | front axle 1.472 m ahead of the actor origin, rear axle 1.389 m behind, track 1.592 m |

## Model parameters (`config/vehicles/mkz2020.json`)

| Parameter | Value | Basis |
| --- | --- | --- |
| `m` | 1696 kg | Read directly. |
| `a`, `b` | 1.172 m, 1.689 m | Derived: the axle positions and the centre of mass 0.30 m ahead of the origin. Wheelbase 2.861 m. |
| `F_brake_max` | 13 kN | Measured: 7.7 m/s² at full brake, 10–20 m/s. The brake torque alone (4 × 800 N·m / 0.355 m) would give only 5.3 m/s². The NMPC's strongest braking is min(this / m, μg) unless `UrbanNMPC::Options::a_min` is set (it is, to −7 m/s²). |
| `delta_max` | 0.95 rad | Measured: the road-wheel angle for full steer at 20 m/s (1.1 rad at 3 m/s); the smaller, so it holds at any speed. |
| `delta_rate_max` | 0.6 rad/s | Design choice. CARLA steers with no rate limit, only a 0.15 s lag. At 1.5 rad/s the offline overtakes (`sim/urban`, whose kinematic plant lags 0.25 s on acceleration and 0.05 s on steering) steer back faster than the lagged plant follows, overshoot the lane and swerve; 1.0 is borderline. |
| `g` | 9.81 m/s² | Constant. |

Throttle, brake and steering responses measured by `scripts/identify_ego.py` (dead zone, acceleration against
speed, steering gain and lag) are in `config/vehicles/mkz2020_actuation.json`, which `expert/mkz_actuation.py` uses.

The body (`ex::Body`, `urban/body.hpp`): 4.89 × 1.84 m, the centre of gravity 0.31 m ahead of the body's centre. The agent widens it
by 0.2 m for the mirrors (`expert/agents/simlingo_nmpc_agent.py`).
