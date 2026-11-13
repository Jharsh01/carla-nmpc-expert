"""What to do when a solve fails: a failed plan is never driven, since its steering means nothing (Bench2Drive route 1:
seven in a row turned the car 21 deg, and the same inputs replayed gave the opposite steering). The last good plan is
followed on for `hold` seconds, as it was made for just this stretch; after that the car also brakes, gently, until a
plan solves again.
"""
import math

HOLD = 0.6  # [s] three plans
BRAKE = -2.0  # [m/s^2]


class PlanHold:
    def __init__(self, hold=HOLD, brake=BRAKE):
        self.hold, self.brake = hold, brake
        self.plan, self.t = None, 0.0  # the plan to follow, and when it was planned
        self.failed_since = None  # when plans began to fail; None while they solve

    def update(self, plan, t):
        """A new plan, made at time t. Before any good plan there is nothing else, so that one stands, under the brake."""
        if plan.success:
            self.plan, self.t, self.failed_since = plan, t, None
            return
        if self.plan is None:
            self.plan, self.t = plan, t
        if self.failed_since is None:
            self.failed_since = t

    def a_limit(self, t):
        """The most acceleration to use at time t."""
        if self.failed_since is None or t - self.failed_since < self.hold:
            return math.inf
        return self.brake
