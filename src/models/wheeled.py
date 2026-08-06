"""Viam movement_sensor: calibrated wheeled odometry for rover encoders."""

from __future__ import annotations

import asyncio
import math
import time
from typing import Any, ClassVar, Dict, List, Mapping, Optional, Sequence, Tuple

from typing_extensions import Self
from viam.components.base import Base
from viam.components.encoder import Encoder
from viam.components.motor import Motor
from viam.components.movement_sensor import GeoPoint, MovementSensor
from viam.errors import NotSupportedError
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import Orientation, ResourceName, Vector3
from viam.proto.component.movementsensor import GetAccuracyResponse
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes, struct_to_dict

from odometry import (
    LutPoint,
    OdomConfig,
    OdomState,
    StepDiagnostics,
    diagnostics_dict,
    parse_lut,
    reset_pose,
    step_odometry,
    suggested_scales_from_straight_run,
)


def _list_strings(fields: Mapping, key: str) -> List[str]:
    if key not in fields:
        return []
    value = fields[key]
    if hasattr(value, "list_value"):
        return [
            item.string_value for item in value.list_value.values if item.string_value
        ]
    return []


def _lut_from_attributes(attrs: Mapping) -> Tuple[LutPoint, ...]:
    raw = attrs.get("ticks_per_rotation_lut")
    if isinstance(raw, list):
        return parse_lut(raw)
    return ()


class CalibratedWheeledOdometry(MovementSensor, EasyResource):
    MODEL: ClassVar[Model] = Model(
        ModelFamily("viam-labs", "calibrated-wheeled-odometry"), "wheeled"
    )

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._base: Optional[Base] = None
        self._left_motor: Optional[Motor] = None
        self._right_motor: Optional[Motor] = None
        self._left_encoder: Optional[Encoder] = None
        self._right_encoder: Optional[Encoder] = None
        self._cfg = OdomConfig(track_width_m=0.3, wheel_circumference_m=0.3)
        self._state = OdomState()
        self._last_step: Optional[StepDiagnostics] = None
        self._interval_s = 0.05
        self._counts_are_revolutions = True
        self._lock = asyncio.Lock()
        self._task: Optional[asyncio.Task] = None
        self._stop = asyncio.Event()
        self._pending_tpr = 0.0
        self._measure_left0_integrated = 0.0
        self._measure_right0_integrated = 0.0
        self._last_error: str = ""
        self._geometry_ready = False

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        self = super().new(config, dependencies)
        attrs = struct_to_dict(config.attributes)
        fields = config.attributes.fields

        base_name = attrs.get("base") or fields["base"].string_value
        left_motors = attrs.get("left_motors") or _list_strings(fields, "left_motors")
        right_motors = attrs.get("right_motors") or _list_strings(fields, "right_motors")
        left_encoders = attrs.get("left_encoders") or _list_strings(
            fields, "left_encoders"
        )
        right_encoders = attrs.get("right_encoders") or _list_strings(
            fields, "right_encoders"
        )

        self._base = dependencies[Base.get_resource_name(str(base_name))]
        self._left_motor = dependencies[Motor.get_resource_name(str(left_motors[0]))]
        self._right_motor = dependencies[Motor.get_resource_name(str(right_motors[0]))]

        if left_encoders and right_encoders:
            self._left_encoder = dependencies[
                Encoder.get_resource_name(str(left_encoders[0]))
            ]
            self._right_encoder = dependencies[
                Encoder.get_resource_name(str(right_encoders[0]))
            ]
            self._counts_are_revolutions = False
        else:
            self._left_encoder = None
            self._right_encoder = None
            self._counts_are_revolutions = True

        interval_ms = float(attrs.get("time_interval_msec", 50) or 50)
        if interval_ms <= 0:
            interval_ms = 50.0
        self._interval_s = interval_ms / 1000.0

        tpr = float(attrs.get("ticks_per_rotation", 0) or 0)
        self._pending_tpr = tpr
        self._cfg = OdomConfig(
            track_width_m=float(attrs.get("width_m", 0) or 0),
            wheel_circumference_m=float(attrs.get("wheel_circumference_m", 0) or 0),
            ticks_per_rotation=tpr if tpr > 0 else 1.0,
            left_scale=float(attrs.get("left_scale", 1.0) or 1.0),
            right_scale=float(attrs.get("right_scale", 1.0) or 1.0),
            max_ticks_per_sec=float(attrs.get("max_ticks_per_sec", 800.0) or 800.0),
            ticks_per_rotation_lut=_lut_from_attributes(attrs),
        )
        self._state = OdomState()
        self._last_step = None
        self._stop = asyncio.Event()
        self._ensure_background_task()
        return self

    def _ensure_background_task(self) -> None:
        if self._task is not None and not self._task.done():
            return
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            return
        self._stop.clear()
        self._task = loop.create_task(self._run_loop())

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> Tuple[Sequence[str], Sequence[str]]:
        attrs = struct_to_dict(config.attributes)
        fields = config.attributes.fields
        base = attrs.get("base") or (
            fields["base"].string_value if "base" in fields else ""
        )
        left_motors = attrs.get("left_motors") or _list_strings(fields, "left_motors")
        right_motors = attrs.get("right_motors") or _list_strings(fields, "right_motors")
        if not base:
            raise ValueError("base is required")
        if not left_motors or not right_motors:
            raise ValueError("left_motors and right_motors are required")
        if len(left_motors) != 1 or len(right_motors) != 1:
            raise ValueError("only one left and one right motor are supported")

        deps: List[str] = [str(base), str(left_motors[0]), str(right_motors[0])]
        left_encoders = attrs.get("left_encoders") or _list_strings(
            fields, "left_encoders"
        )
        right_encoders = attrs.get("right_encoders") or _list_strings(
            fields, "right_encoders"
        )
        if left_encoders or right_encoders:
            if len(left_encoders) != 1 or len(right_encoders) != 1:
                raise ValueError(
                    "left_encoders and right_encoders must each list exactly one encoder"
                )
            deps.extend([str(left_encoders[0]), str(right_encoders[0])])
        return deps, []

    async def _refresh_base_geometry(self) -> None:
        assert self._base is not None
        props = await self._base.get_properties()
        width = float(props.width_meters)
        circ = float(props.wheel_circumference_meters)
        if self._cfg.track_width_m <= 0 and width > 0:
            self._cfg.track_width_m = width
        if self._cfg.wheel_circumference_m <= 0 and circ > 0:
            self._cfg.wheel_circumference_m = circ
        if self._cfg.track_width_m <= 0 or self._cfg.wheel_circumference_m <= 0:
            raise RuntimeError(
                "base width_meters and wheel_circumference_meters must be > 0 "
                "(set on the base, or pass width_m / wheel_circumference_m)"
            )
        self._geometry_ready = True

    async def _resolve_ticks_per_rotation(self) -> None:
        if self._pending_tpr > 0:
            self._cfg.ticks_per_rotation = self._pending_tpr
            return
        if not self._counts_are_revolutions and self._cfg.ticks_per_rotation <= 1.0:
            self.logger.warning(
                "ticks_per_rotation not set while using encoders; "
                "odometry distances will be wrong until configured"
            )

    async def _read_counts(self) -> Tuple[float, float]:
        if self._left_encoder is not None and self._right_encoder is not None:
            (left, _), (right, _) = await asyncio.gather(
                self._left_encoder.get_position(),
                self._right_encoder.get_position(),
            )
            return float(left), float(right)
        assert self._left_motor is not None and self._right_motor is not None
        left, right = await asyncio.gather(
            self._left_motor.get_position(),
            self._right_motor.get_position(),
        )
        return float(left), float(right)

    async def _read_powers(self) -> Tuple[float, float]:
        assert self._left_motor is not None and self._right_motor is not None
        (_, left_pwr), (_, right_pwr) = await asyncio.gather(
            self._left_motor.is_powered(),
            self._right_motor.is_powered(),
        )
        return float(left_pwr), float(right_pwr)

    async def _run_loop(self) -> None:
        try:
            await self._refresh_base_geometry()
            await self._resolve_ticks_per_rotation()
        except Exception as exc:  # noqa: BLE001
            self._last_error = str(exc)
            self.logger.error("calibrated odometry init failed: %s", exc)

        last_t = time.monotonic()
        geo_check_at = 0.0
        while not self._stop.is_set():
            try:
                await asyncio.sleep(self._interval_s)
                now = time.monotonic()
                dt = max(now - last_t, 1e-6)
                last_t = now
                if not self._geometry_ready or now >= geo_check_at:
                    try:
                        await self._refresh_base_geometry()
                        geo_check_at = now + 5.0
                    except Exception as exc:  # noqa: BLE001
                        self._last_error = str(exc)
                        continue
                left_c, right_c = await self._read_counts()
                left_p, right_p = await self._read_powers()
                async with self._lock:
                    self._state, self._last_step = step_odometry(
                        self._state,
                        self._cfg,
                        left_ticks=left_c,
                        right_ticks=right_c,
                        left_power=left_p,
                        right_power=right_p,
                        dt_s=dt,
                        counts_are_revolutions=self._counts_are_revolutions,
                    )
                self._last_error = ""
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # noqa: BLE001
                self._last_error = str(exc)
                self.logger.error("calibrated odometry step failed: %s", exc)

    async def close(self) -> None:
        self._stop.set()
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except (asyncio.CancelledError, Exception):  # noqa: BLE001
                pass
            self._task = None

    async def get_linear_velocity(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Vector3:
        self._ensure_background_task()
        async with self._lock:
            return Vector3(x=self._state.vx_mps, y=self._state.vy_mps, z=0.0)

    async def get_angular_velocity(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Vector3:
        self._ensure_background_task()
        async with self._lock:
            return Vector3(x=0.0, y=0.0, z=self._state.wz_deg_s)

    async def get_orientation(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Orientation:
        self._ensure_background_task()
        async with self._lock:
            yaw_deg = math.degrees(self._state.yaw_rad) % 360.0
            return Orientation(o_x=0.0, o_y=0.0, o_z=1.0, theta=yaw_deg)

    async def get_position(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Tuple[GeoPoint, float]:
        self._ensure_background_task()
        async with self._lock:
            x, y = self._state.x, self._state.y
        # Builtin relative mode uses geo.NewPoint(Y, X) → lat=Y, lng=X.
        return GeoPoint(latitude=y, longitude=x), 0.0

    async def get_compass_heading(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> float:
        raise NotSupportedError(
            "compass heading not supported; use get_orientation for odom yaw"
        )

    async def get_linear_acceleration(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Vector3:
        raise NotSupportedError("linear acceleration not supported")

    async def get_properties(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> MovementSensor.Properties:
        return MovementSensor.Properties(
            linear_velocity_supported=True,
            angular_velocity_supported=True,
            orientation_supported=True,
            position_supported=True,
            compass_heading_supported=False,
            linear_acceleration_supported=False,
        )

    async def get_accuracy(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> MovementSensor.Accuracy:
        return GetAccuracyResponse()

    async def get_readings(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Mapping[str, ValueTypes]:
        self._ensure_background_task()
        lin = await self.get_linear_velocity(extra=extra, timeout=timeout)
        ang = await self.get_angular_velocity(extra=extra, timeout=timeout)
        orient = await self.get_orientation(extra=extra, timeout=timeout)
        async with self._lock:
            x, y = self._state.x, self._state.y
        return {
            "linear_velocity": {"x": lin.x, "y": lin.y, "z": lin.z},
            "angular_velocity": {"x": ang.x, "y": ang.y, "z": ang.z},
            "orientation": {
                "o_x": orient.o_x,
                "o_y": orient.o_y,
                "o_z": orient.o_z,
                "theta": orient.theta,
            },
            "position_meters_X": x,
            "position_meters_Y": y,
        }

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Mapping[str, ValueTypes]:
        self._ensure_background_task()
        cmd = command.get("command") or command.get("cmd")
        if cmd is None and len(command) == 1:
            only_key = next(iter(command.keys()))
            if only_key in {
                "reset_pose",
                "get_diagnostics",
                "set_scales",
                "measure_straight_run",
                "reset",
            }:
                cmd = only_key

        if cmd == "reset_pose" or command.get("reset") is True:
            async with self._lock:
                reset_pose(self._state)
                self._state.last_left_ticks = None
                self._state.last_right_ticks = None
            return {"reset_pose": True}

        if cmd == "get_diagnostics":
            async with self._lock:
                out = diagnostics_dict(self._state, self._cfg, self._last_step)
            out["last_error"] = self._last_error
            out["counts_are_revolutions"] = self._counts_are_revolutions
            out["interval_s"] = self._interval_s
            return out

        if cmd == "set_scales":
            left = command.get("left_scale")
            right = command.get("right_scale")
            async with self._lock:
                if left is not None:
                    self._cfg.left_scale = float(left)
                if right is not None:
                    self._cfg.right_scale = float(right)
                return {
                    "left_scale": self._cfg.left_scale,
                    "right_scale": self._cfg.right_scale,
                }

        if cmd == "measure_straight_run":
            action = str(command.get("action", "start"))
            if action == "start":
                async with self._lock:
                    self._measure_left0_integrated = self._state.integrated_left_ticks
                    self._measure_right0_integrated = self._state.integrated_right_ticks
                return {"action": "start", "ok": True}

            if action == "finish":
                measured = float(command.get("measured_distance_m", 0) or 0)
                if measured <= 0:
                    raise ValueError("measured_distance_m must be > 0")
                async with self._lock:
                    left_ticks = (
                        self._state.integrated_left_ticks
                        - self._measure_left0_integrated
                    )
                    right_ticks = (
                        self._state.integrated_right_ticks
                        - self._measure_right0_integrated
                    )
                    left_s, right_s = suggested_scales_from_straight_run(
                        measured_distance_m=measured,
                        left_ticks=left_ticks,
                        right_ticks=right_ticks,
                        ticks_per_rotation=self._cfg.ticks_per_rotation,
                        wheel_circumference_m=self._cfg.wheel_circumference_m,
                        current_left_scale=self._cfg.left_scale,
                        current_right_scale=self._cfg.right_scale,
                    )
                    apply = bool(command.get("apply", False))
                    if apply:
                        self._cfg.left_scale = left_s
                        self._cfg.right_scale = right_s
                return {
                    "action": "finish",
                    "measured_distance_m": measured,
                    "left_ticks": left_ticks,
                    "right_ticks": right_ticks,
                    "suggested_left_scale": left_s,
                    "suggested_right_scale": right_s,
                    "applied": apply,
                    "note": (
                        "Persist left_scale/right_scale in the module config "
                        "to keep after restart"
                    ),
                }

            raise ValueError("measure_straight_run action must be 'start' or 'finish'")

        raise ValueError(
            "unknown command; supported: reset_pose, get_diagnostics, set_scales, "
            "measure_straight_run"
        )
