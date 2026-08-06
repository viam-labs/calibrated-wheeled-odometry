"""Unit tests for calibrated wheeled-odometry math."""

from __future__ import annotations

import math

import pytest

from odometry import (
    LutPoint,
    OdomConfig,
    OdomState,
    direction_from_power,
    interpolate_ticks_per_rotation,
    parse_lut,
    reject_spike,
    reset_pose,
    step_odometry,
    suggested_scales_from_straight_run,
    ticks_to_meters,
)


def test_direction_from_power_holds_last_sign_when_coasting():
    assert direction_from_power(0.5, last_sign=1.0) == 1.0
    assert direction_from_power(-0.5, last_sign=1.0) == -1.0
    assert direction_from_power(0.0, last_sign=-1.0) == -1.0
    assert direction_from_power(0.01, last_sign=-1.0) == -1.0  # inside deadband


def test_reject_spike_zeros_impossible_rate():
    ok, rejected = reject_spike(10.0, dt_s=0.05, max_ticks_per_sec=800.0)
    assert ok == 10.0 and not rejected
    filtered, rejected = reject_spike(100.0, dt_s=0.05, max_ticks_per_sec=800.0)
    # 100/0.05 = 2000 > 800
    assert filtered == 0.0 and rejected


def test_lut_interpolation():
    lut = (
        LutPoint(0.2, 400.0),
        LutPoint(0.4, 500.0),
        LutPoint(0.6, 600.0),
    )
    assert interpolate_ticks_per_rotation(lut, 0.1, default=1.0) == 400.0
    assert interpolate_ticks_per_rotation(lut, 0.3, default=1.0) == 450.0
    assert interpolate_ticks_per_rotation(lut, 0.9, default=1.0) == 600.0
    assert interpolate_ticks_per_rotation((), 0.3, default=480.0) == 480.0


def test_parse_lut_sorts_by_speed():
    lut = parse_lut(
        [
            {"speed_mps": 0.5, "ticks_per_rotation": 500},
            {"speed_mps": 0.2, "ticks_per_rotation": 400},
        ]
    )
    assert lut[0].speed_mps == 0.2
    assert lut[1].ticks_per_rotation == 500.0


def test_straight_forward_integration_y_increases():
    cfg = OdomConfig(
        track_width_m=0.3,
        wheel_circumference_m=0.5,
        ticks_per_rotation=100.0,
        max_ticks_per_sec=1e6,
    )
    state = OdomState()
    # First sample anchors.
    state, _ = step_odometry(
        state,
        cfg,
        left_ticks=0.0,
        right_ticks=0.0,
        left_power=0.5,
        right_power=0.5,
        dt_s=0.05,
    )
    # One full rotation each wheel → 0.5 m each → ds = 0.5 m
    state, diag = step_odometry(
        state,
        cfg,
        left_ticks=100.0,
        right_ticks=100.0,
        left_power=0.5,
        right_power=0.5,
        dt_s=0.05,
    )
    assert diag.ds_m == pytest.approx(0.5)
    assert diag.dtheta_rad == pytest.approx(0.0)
    assert state.y == pytest.approx(0.5)
    assert state.x == pytest.approx(0.0)
    assert state.vy_mps == pytest.approx(0.5 / 0.05)


def test_pivot_in_place_changes_yaw_only():
    cfg = OdomConfig(
        track_width_m=0.4,
        wheel_circumference_m=0.4,
        ticks_per_rotation=100.0,
        max_ticks_per_sec=1e6,
    )
    state = OdomState()
    state, _ = step_odometry(
        state,
        cfg,
        left_ticks=0.0,
        right_ticks=0.0,
        left_power=-0.5,
        right_power=0.5,
        dt_s=0.05,
    )
    # Left -100 ticks (backward), right +100 → left_m=-0.4, right_m=+0.4
    # With single-pin we take |Δ| then apply power sign.
    state, diag = step_odometry(
        state,
        cfg,
        left_ticks=100.0,
        right_ticks=100.0,
        left_power=-0.5,
        right_power=0.5,
        dt_s=0.05,
    )
    assert diag.ds_m == pytest.approx(0.0)
    # dθ = (0.4 - (-0.4)) / 0.4 = 2.0 rad
    assert diag.dtheta_rad == pytest.approx(2.0)
    assert state.x == pytest.approx(0.0)
    assert state.y == pytest.approx(0.0)


def test_spike_does_not_move_pose():
    cfg = OdomConfig(
        track_width_m=0.3,
        wheel_circumference_m=0.5,
        ticks_per_rotation=100.0,
        max_ticks_per_sec=200.0,
    )
    state = OdomState()
    state, _ = step_odometry(
        state,
        cfg,
        left_ticks=0.0,
        right_ticks=0.0,
        left_power=0.5,
        right_power=0.5,
        dt_s=0.05,
    )
    state, diag = step_odometry(
        state,
        cfg,
        left_ticks=1000.0,  # 1000/0.05 = 20k ticks/s → reject
        right_ticks=1000.0,
        left_power=0.5,
        right_power=0.5,
        dt_s=0.05,
    )
    assert diag.left_rejected and diag.right_rejected
    assert state.x == 0.0 and state.y == 0.0
    assert state.rejected_spikes == 2


def test_left_right_scale_bias():
    cfg = OdomConfig(
        track_width_m=0.3,
        wheel_circumference_m=1.0,
        ticks_per_rotation=1.0,
        left_scale=1.0,
        right_scale=2.0,
        max_ticks_per_sec=1e6,
    )
    state = OdomState()
    state, _ = step_odometry(
        state,
        cfg,
        left_ticks=0.0,
        right_ticks=0.0,
        left_power=0.5,
        right_power=0.5,
        dt_s=0.1,
        counts_are_revolutions=True,
    )
    state, diag = step_odometry(
        state,
        cfg,
        left_ticks=1.0,
        right_ticks=1.0,
        left_power=0.5,
        right_power=0.5,
        dt_s=0.1,
        counts_are_revolutions=True,
    )
    # left_m=1, right_m=2 → ds=1.5, dθ=(2-1)/0.3
    assert diag.ds_m == pytest.approx(1.5)
    assert diag.dtheta_rad == pytest.approx(1.0 / 0.3)


def test_reset_pose_zeros_integration():
    state = OdomState(x=1.0, y=2.0, yaw_rad=0.5, vy_mps=0.3)
    reset_pose(state)
    assert state.x == state.y == state.yaw_rad == state.vy_mps == 0.0


def test_suggested_scales_from_straight_run():
    # With scale=1, 100 ticks @ 100 tpr @ 1m circ → 1.0 m implied; measured 2.0 → scale 2
    left, right = suggested_scales_from_straight_run(
        measured_distance_m=2.0,
        left_ticks=100.0,
        right_ticks=100.0,
        ticks_per_rotation=100.0,
        wheel_circumference_m=1.0,
        current_left_scale=1.0,
        current_right_scale=1.0,
    )
    assert left == pytest.approx(2.0)
    assert right == pytest.approx(2.0)


def test_ticks_to_meters():
    assert ticks_to_meters(
        50.0, ticks_per_rotation=100.0, wheel_circumference_m=0.4, scale=1.0
    ) == pytest.approx(0.2)
