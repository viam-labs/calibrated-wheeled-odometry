# Calibrated Wheeled Odometry

Viam `movement_sensor` module that replaces builtin `wheeled-odometry` with
calibration and noise handling aimed at **Viam Rover-style single-pin encoders**.

## Why this exists

Stock wheeled odometry assumes clean, direction-aware encoder ticks. Viam Rover
motors use single-pin encoders that:

1. **Cannot sense direction** — tick count only increases; direction must be
   inferred from commanded motor power.
2. **Pick up motor electrical noise** — tick rates explode when motors are
   powered. An RC filter on the encoder lines is the real hardware fix; this
   module rejects impossible tick-rate spikes in software.
3. **Have L/R and speed-dependent scale error** — per-wheel scales and an
   optional speed → ticks_per_rotation lookup table correct systematic bias.

This module does **not** wrap motors with a PWM feed-forward LUT (v1). Point
nav-stack (or anything else) at this sensor instead of `wheeled-odometry`.

## Machine configuration

```json
{
  "name": "odometry",
  "api": "rdk:component:movement_sensor",
  "model": "viam-labs:calibrated-wheeled-odometry:wheeled",
  "attributes": {
    "base": "viam_base",
    "left_motors": ["left"],
    "right_motors": ["right"],
    "time_interval_msec": 50,
    "max_ticks_per_sec": 800,
    "left_scale": 1.0,
    "right_scale": 1.0,
    "ticks_per_rotation": 0
  },
  "depends_on": ["viam_base", "left", "right"]
}
```

| Attribute | Default | Notes |
|---|---|---|
| `base` | required | Provides `width_meters` / `wheel_circumference_meters` |
| `left_motors` / `right_motors` | required | One each; power sign + (by default) position |
| `left_encoders` / `right_encoders` | optional | If set, read ticks from encoders instead of motor position |
| `time_interval_msec` | `50` | Update period (builtin wheeled-odometry defaults to 500) |
| `max_ticks_per_sec` | `800` | Spike rejection threshold |
| `left_scale` / `right_scale` | `1.0` | Per-wheel distance multipliers |
| `ticks_per_rotation` | `0` | `0` → treat motor positions as revolutions (TPR=1). Required when using encoders |
| `ticks_per_rotation_lut` | `[]` | `[{ "speed_mps": 0.2, "ticks_per_rotation": 480 }, ...]` |
| `width_m` / `wheel_circumference_m` | from base | Optional overrides |

Readings match builtin wheeled-odometry (`position_meters_X` / `Y`, forward
velocity on `linear_velocity.y`, `angular_velocity.z` in deg/s) so nav-stack
works without code changes — set `movement_sensor` to this component name.

## DoCommand calibration

```json
{ "command": "reset_pose" }
{ "command": "get_diagnostics" }
{ "command": "set_scales", "left_scale": 1.02, "right_scale": 0.98 }
{ "command": "measure_straight_run", "action": "start" }
{ "command": "measure_straight_run", "action": "finish", "measured_distance_m": 2.0, "apply": true }
```

Straight-run workflow: `start` → drive roughly straight on the floor → measure
tape distance → `finish` with `measured_distance_m`. Copy suggested scales into
the module config to persist.

**Limitation:** single-pin direction is inferred from motor power. Pushing the
robot by hand while motors are stopped will desynchronize odometry — call
`reset_pose` or re-localize afterward.

## Hardware note

Software spike rejection cannot recover a signal already destroyed by motor
noise. For reliable odometry, add an RC low-pass on each encoder line (e.g.
100 Ω series + 100 nF to ground, with a 10 kΩ pull-up to 3.3 V), as described in
[Mike Likes Robots' Viam Rover write-up](https://mikelikesrobots.github.io/blog/autonomous-viam-rover/).

## Develop / test

```bash
./setup.sh
source venv/bin/activate
pip install -r requirements.txt pytest
PYTHONPATH=src pytest -q
./build.sh   # produces module.tar.gz for viam module upload
```
