"""Helpers for the feed-override step tests: signed pulse counting, leg bounds and feedhold confirmation.

Pure functions (no board access) so the offline tests can prove what the hardware tests would miss.

DIR convention (stepper.c st_prep_line and the motor load blocks): positive travel sets
direction = DIRECTION_CW ^ polarity, negative travel DIRECTION_CCW ^ polarity, and DIRECTION_CW (0,
canonical_machine.h) drives the DIR pin low. So the pin level during positive travel equals the motor's $<m>po.
"""
import time

HOLD_COMPLETE = 6          # cmFeedholdState FEEDHOLD_HOLD, reported as {"hold":6}


class HoldNotReached(AssertionError):
    pass


def dir_level(positive, polarity):
    """DIR pin level the firmware drives for travel in the given direction with $<m>po = polarity."""
    return (0 if positive else 1) ^ (1 if polarity else 0)


def signed_steps(rises, dir_edges, dir_initial, forward_level):
    """(net steps in the forward direction, steps the other way, forward pulse times).

    rises: step pulse rise times; dir_edges: [(t, level)] DIR transitions; dir_initial: DIR level at capture start
    (required: a missing level is an error, never assumed); forward_level: the DIR level of the commanded direction.
    Each pulse counts by the DIR level at that moment, so a move in the wrong direction comes out negative.
    """
    if dir_initial not in (0, 1):
        raise ValueError(f"DIR initial level unknown: {dir_initial!r}")
    level, i, fwd, back = dir_initial, 0, [], 0
    for t in rises:
        while i < len(dir_edges) and dir_edges[i][0] <= t:
            level = dir_edges[i][1]
            i += 1
        if level == forward_level:
            fwd.append(t)
        else:
            back += 1
    return len(fwd) - back, back, fwd


def within(steps, target, tol=1):
    """A signed step count within tol of a signed target (direction matters)."""
    return abs(steps - target) <= tol


def wait_for_hold(read_hold, timeout=5.0, poll=0.05, clock=time.monotonic, sleep=time.sleep):
    """Poll read_hold() until it reports a completed hold; return the seconds it took, or raise HoldNotReached."""
    t0 = clock()
    seen = []
    while True:
        value = read_hold()
        seen.append(value)
        if value == HOLD_COMPLETE:
            return clock() - t0
        if clock() - t0 >= timeout:
            raise HoldNotReached(f"feedhold not complete within {timeout} s (hold states seen: {seen[-10:]})")
        sleep(poll)
