"""Pure wheeled-odometry math for calibrated single-pin rover encoders.

No Viam I/O — unit-testable. Handles:
* max tick-rate spike rejection
* direction from commanded motor power (single-pin encoders)
* per-wheel scale factors
* optional speed → ticks_per_rotation LUT for sensing
* differential-drive pose / twist integration (Viam wheeled-odometry frame:
  +Y forward, yaw about +Z, angular velocity in deg/s for the API layer)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Mapping, Optional, Sequence, Tuple


TWO_PI = 2.0 * math.pi


@dataclass(frozen=True)
class LutPoint:
    """One entry in a speed → ticks_per_rotation lookup table."""

    speed_mps: float
    ticks_per_rotation: float


@dataclass
class OdomConfig:
    track_width_m: float
    wheel_circumference_m: float
    ticks_per_rotation: float = 1.0
    left_scale: float = 1.0
    right_scale: float = 1.0
    max_ticks_per_sec: float = 800.0
    ticks_per_rotation_lut: Tuple[LutPoint, ...] = ()

    def effective_ticks_per_rotation(self, speed_mps: float) -> float:
        """Interpolate TPR from the LUT, falling back to the baseline."""
        return interpolate_ticks_per_rotation(
            self.ticks_per_rotation_lut,
            abs(float(speed_mps)),
            default=self.ticks_per_rotation,
        )


@dataclass
class OdomState:
    """Integrated pose in the builtin wheeled-odometry body/world convention.

    ``x`` / ``y`` are meters in the odom frame used by ``rdk:builtin:wheeled-odometry``
    (+Y forward at yaw 0). ``yaw_rad`` is CCW from +Y toward +X when use_compass
    is false (same as the Go driver without compass mode).
    """

    x: float = 0.0
    y: float = 0.0
    yaw_rad: float = 0.0
    vx_mps: float = 0.0  # body +X (lateral); usually 0 for diff-drive
    vy_mps: float = 0.0  # body +Y (forward) — matches builtin linearVelocity.Y
    wz_deg_s: float = 0.0
    left_sign: float = 1.0
    right_sign: float = 1.0
    rejected_spikes: int = 0
    last_left_ticks: Optional[float] = None
    last_right_ticks: Optional[float] = None
    last_left_delta_ticks: float = 0.0
    last_right_delta_ticks: float = 0.0
    last_dt_s: float = 0.0
    integrated_left_ticks: float = 0.0
    integrated_right_ticks: float = 0.0


@dataclass
class StepDiagnostics:
    left_raw_delta: float
    right_raw_delta: float
    left_signed_delta: float
    right_signed_delta: float
    left_rejected: bool
    right_rejected: bool
    left_sign: float
    right_sign: float
    effective_tpr: float
    ds_m: float
    dtheta_rad: float


def parse_lut(entries: Sequence[Mapping] | None) -> Tuple[LutPoint, ...]:
    """Parse config list ``[{speed_mps, ticks_per_rotation}, ...]``."""
    if not entries:
        return ()
    points: List[LutPoint] = []
    for entry in entries:
        if not isinstance(entry, Mapping):
            continue
        points.append(
            LutPoint(
                speed_mps=float(entry["speed_mps"]),
                ticks_per_rotation=float(entry["ticks_per_rotation"]),
            )
        )
    points.sort(key=lambda p: p.speed_mps)
    return tuple(points)


def interpolate_ticks_per_rotation(
    lut: Sequence[LutPoint],
    speed_mps: float,
    *,
    default: float,
) -> float:
    """Piecewise-linear TPR at ``speed_mps``; clamp outside the table."""
    if not lut:
        return float(default)
    if speed_mps <= lut[0].speed_mps:
        return float(lut[0].ticks_per_rotation)
    if speed_mps >= lut[-1].speed_mps:
        return float(lut[-1].ticks_per_rotation)
    for i in range(1, len(lut)):
        a, b = lut[i - 1], lut[i]
        if speed_mps <= b.speed_mps:
            span = b.speed_mps - a.speed_mps
            if span <= 0:
                return float(b.ticks_per_rotation)
            t = (speed_mps - a.speed_mps) / span
            return float(
                a.ticks_per_rotation
                + t * (b.ticks_per_rotation - a.ticks_per_rotation)
            )
    return float(default)


def direction_from_power(
    power: float,
    *,
    last_sign: float,
    deadband: float = 0.02,
) -> float:
    """Infer wheel travel sign from commanded motor power.

    Single-pin encoders only report magnitude. Hold the last non-zero sign when
    coasting so brief zero-power samples do not flip direction. Hand-pushing the
    robot while powered off will still desynchronize odometry.
    """
    if power > deadband:
        return 1.0
    if power < -deadband:
        return -1.0
    if last_sign == 0.0:
        return 1.0
    return 1.0 if last_sign > 0 else -1.0


def reject_spike(
    delta_ticks: float,
    dt_s: float,
    max_ticks_per_sec: float,
) -> Tuple[float, bool]:
    """Zero out an impossible tick jump; return (filtered_delta, was_rejected)."""
    if dt_s <= 0 or max_ticks_per_sec <= 0:
        return delta_ticks, False
    rate = abs(delta_ticks) / dt_s
    if rate > max_ticks_per_sec:
        return 0.0, True
    return delta_ticks, False


def ticks_to_meters(
    delta_ticks: float,
    *,
    ticks_per_rotation: float,
    wheel_circumference_m: float,
    scale: float,
) -> float:
    if ticks_per_rotation == 0:
        return 0.0
    return (delta_ticks / ticks_per_rotation) * wheel_circumference_m * scale


def revolutions_to_meters(
    delta_revs: float,
    *,
    wheel_circumference_m: float,
    scale: float,
    base_tpr: float,
    effective_tpr: float,
) -> float:
    """Convert motor-reported revolution delta to meters.

    When an effective TPR differs from the motor's configured TPR, scale the
    reported revolutions so distance reflects the calibrated tick density.
    """
    if effective_tpr <= 0 or base_tpr <= 0:
        correction = 1.0
    else:
        correction = base_tpr / effective_tpr
    return delta_revs * wheel_circumference_m * scale * correction


def step_odometry(
    state: OdomState,
    cfg: OdomConfig,
    *,
    left_ticks: float,
    right_ticks: float,
    left_power: float,
    right_power: float,
    dt_s: float,
    counts_are_revolutions: bool = False,
) -> Tuple[OdomState, StepDiagnostics]:
    """One odometry update from absolute tick (or revolution) counts.

    When ``counts_are_revolutions`` is True, ``left_ticks``/``right_ticks`` are
    motor ``GetPosition`` values in revolutions (builtin wheeled-odometry style).
    Otherwise they are raw encoder ticks.
    """
    if state.last_left_ticks is None or state.last_right_ticks is None:
        state.last_left_ticks = left_ticks
        state.last_right_ticks = right_ticks
        state.left_sign = direction_from_power(left_power, last_sign=state.left_sign)
        state.right_sign = direction_from_power(right_power, last_sign=state.right_sign)
        diag = StepDiagnostics(
            left_raw_delta=0.0,
            right_raw_delta=0.0,
            left_signed_delta=0.0,
            right_signed_delta=0.0,
            left_rejected=False,
            right_rejected=False,
            left_sign=state.left_sign,
            right_sign=state.right_sign,
            effective_tpr=cfg.ticks_per_rotation,
            ds_m=0.0,
            dtheta_rad=0.0,
        )
        return state, diag

    raw_l = left_ticks - state.last_left_ticks
    raw_r = right_ticks - state.last_right_ticks
    state.last_left_ticks = left_ticks
    state.last_right_ticks = right_ticks
    state.last_dt_s = dt_s

    # Single-pin: magnitude from |Δ|, sign from commanded power.
    # If the underlying driver already signs revolutions (quadrature), |Δ| still
    # works when combined with power sign; conflicting signs mean the robot was
    # commanded opposite to reported motion and we trust the command.
    left_sign = direction_from_power(left_power, last_sign=state.left_sign)
    right_sign = direction_from_power(right_power, last_sign=state.right_sign)
    state.left_sign = left_sign
    state.right_sign = right_sign

    mag_l = abs(raw_l)
    mag_r = abs(raw_r)

    # Spike filter operates in tick-equivalent units.
    if counts_are_revolutions:
        tick_equiv_l = mag_l * cfg.ticks_per_rotation
        tick_equiv_r = mag_r * cfg.ticks_per_rotation
    else:
        tick_equiv_l = mag_l
        tick_equiv_r = mag_r

    filt_l, rej_l = reject_spike(tick_equiv_l, dt_s, cfg.max_ticks_per_sec)
    filt_r, rej_r = reject_spike(tick_equiv_r, dt_s, cfg.max_ticks_per_sec)
    if rej_l:
        state.rejected_spikes += 1
        filt_l = 0.0
    if rej_r:
        state.rejected_spikes += 1
        filt_r = 0.0

    signed_tick_l = left_sign * filt_l
    signed_tick_r = right_sign * filt_r
    state.last_left_delta_ticks = signed_tick_l
    state.last_right_delta_ticks = signed_tick_r
    state.integrated_left_ticks += signed_tick_l
    state.integrated_right_ticks += signed_tick_r

    # Estimate speed for LUT from previous vy (body forward).
    speed_est = abs(state.vy_mps)
    tpr = cfg.effective_ticks_per_rotation(speed_est)

    if counts_are_revolutions:
        # Convert filtered tick-equivalents back to revolutions for distance.
        delta_revs_l = (filt_l / cfg.ticks_per_rotation) if cfg.ticks_per_rotation else 0.0
        delta_revs_r = (filt_r / cfg.ticks_per_rotation) if cfg.ticks_per_rotation else 0.0
        left_m = revolutions_to_meters(
            left_sign * delta_revs_l,
            wheel_circumference_m=cfg.wheel_circumference_m,
            scale=cfg.left_scale,
            base_tpr=cfg.ticks_per_rotation,
            effective_tpr=tpr,
        )
        right_m = revolutions_to_meters(
            right_sign * delta_revs_r,
            wheel_circumference_m=cfg.wheel_circumference_m,
            scale=cfg.right_scale,
            base_tpr=cfg.ticks_per_rotation,
            effective_tpr=tpr,
        )
    else:
        left_m = ticks_to_meters(
            signed_tick_l,
            ticks_per_rotation=tpr,
            wheel_circumference_m=cfg.wheel_circumference_m,
            scale=cfg.left_scale,
        )
        right_m = ticks_to_meters(
            signed_tick_r,
            ticks_per_rotation=tpr,
            wheel_circumference_m=cfg.wheel_circumference_m,
            scale=cfg.right_scale,
        )

    if cfg.track_width_m <= 0:
        ds = 0.5 * (left_m + right_m)
        dtheta = 0.0
    else:
        ds = 0.5 * (left_m + right_m)
        dtheta = (right_m - left_m) / cfg.track_width_m

    # Match builtin wheeled-odometry integration (non-compass):
    # yaw accumulates, X += -ds*sin(yaw), Y += ds*cos(yaw).
    state.yaw_rad = (state.yaw_rad + dtheta) % TWO_PI
    if state.yaw_rad < 0:
        state.yaw_rad += TWO_PI
    angle = state.yaw_rad
    state.x += -ds * math.sin(angle)
    state.y += ds * math.cos(angle)

    if dt_s > 0:
        state.vy_mps = ds / dt_s
        state.vx_mps = 0.0
        state.wz_deg_s = math.degrees(dtheta) / dt_s
    else:
        state.vy_mps = 0.0
        state.vx_mps = 0.0
        state.wz_deg_s = 0.0

    diag = StepDiagnostics(
        left_raw_delta=raw_l,
        right_raw_delta=raw_r,
        left_signed_delta=signed_tick_l,
        right_signed_delta=signed_tick_r,
        left_rejected=rej_l,
        right_rejected=rej_r,
        left_sign=left_sign,
        right_sign=right_sign,
        effective_tpr=tpr,
        ds_m=ds,
        dtheta_rad=dtheta,
    )
    return state, diag


def reset_pose(state: OdomState) -> OdomState:
    """Zero integrated pose and twist; keep tick anchors and scales elsewhere."""
    state.x = 0.0
    state.y = 0.0
    state.yaw_rad = 0.0
    state.vx_mps = 0.0
    state.vy_mps = 0.0
    state.wz_deg_s = 0.0
    state.integrated_left_ticks = 0.0
    state.integrated_right_ticks = 0.0
    return state


def suggested_scales_from_straight_run(
    *,
    measured_distance_m: float,
    left_ticks: float,
    right_ticks: float,
    ticks_per_rotation: float,
    wheel_circumference_m: float,
    current_left_scale: float = 1.0,
    current_right_scale: float = 1.0,
) -> Tuple[float, float]:
    """Suggest new left/right scales after a measured straight run.

    Uses the average of the two wheel paths as the estimated travel; each side
    is scaled so its implied distance matches the tape measurement. Relative
    L/R imbalance is preserved via per-side correction against the mean.
    """
    if (
        measured_distance_m <= 0
        or ticks_per_rotation <= 0
        or wheel_circumference_m <= 0
    ):
        return current_left_scale, current_right_scale

    left_m = ticks_to_meters(
        abs(left_ticks),
        ticks_per_rotation=ticks_per_rotation,
        wheel_circumference_m=wheel_circumference_m,
        scale=current_left_scale,
    )
    right_m = ticks_to_meters(
        abs(right_ticks),
        ticks_per_rotation=ticks_per_rotation,
        wheel_circumference_m=wheel_circumference_m,
        scale=current_right_scale,
    )
    mean_m = 0.5 * (left_m + right_m)
    if mean_m <= 1e-9:
        return current_left_scale, current_right_scale

    # Global distance correction from mean path length.
    global_corr = measured_distance_m / mean_m
    # Optionally nudge sides toward each other if they disagree; keep simple:
    # apply the same global correction to both (straight-run distance cal).
    return (
        current_left_scale * global_corr,
        current_right_scale * global_corr,
    )


def diagnostics_dict(
    state: OdomState,
    cfg: OdomConfig,
    last_step: Optional[StepDiagnostics] = None,
) -> dict:
    out = {
        "x_m": state.x,
        "y_m": state.y,
        "yaw_rad": state.yaw_rad,
        "yaw_deg": math.degrees(state.yaw_rad),
        "vy_mps": state.vy_mps,
        "wz_deg_s": state.wz_deg_s,
        "left_sign": state.left_sign,
        "right_sign": state.right_sign,
        "rejected_spikes": state.rejected_spikes,
        "left_scale": cfg.left_scale,
        "right_scale": cfg.right_scale,
        "ticks_per_rotation": cfg.ticks_per_rotation,
        "max_ticks_per_sec": cfg.max_ticks_per_sec,
        "track_width_m": cfg.track_width_m,
        "wheel_circumference_m": cfg.wheel_circumference_m,
        "integrated_left_ticks": state.integrated_left_ticks,
        "integrated_right_ticks": state.integrated_right_ticks,
        "last_left_delta_ticks": state.last_left_delta_ticks,
        "last_right_delta_ticks": state.last_right_delta_ticks,
        "last_dt_s": state.last_dt_s,
    }
    if last_step is not None:
        out.update(
            {
                "last_left_raw_delta": last_step.left_raw_delta,
                "last_right_raw_delta": last_step.right_raw_delta,
                "last_left_rejected": last_step.left_rejected,
                "last_right_rejected": last_step.right_rejected,
                "effective_tpr": last_step.effective_tpr,
                "last_ds_m": last_step.ds_m,
                "last_dtheta_rad": last_step.dtheta_rad,
            }
        )
    return out
