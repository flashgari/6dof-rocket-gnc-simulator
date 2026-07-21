"""Time-varying propulsion and mass-property models."""

from __future__ import annotations

from dataclasses import dataclass

from .math3d import Vector
from .models import RocketParams


def lerp(a: float, b: float, fraction: float) -> float:
    return a + (b - a) * fraction


def lerp_vec(a: Vector, b: Vector, fraction: float) -> Vector:
    return (
        lerp(a[0], b[0], fraction),
        lerp(a[1], b[1], fraction),
        lerp(a[2], b[2], fraction),
    )


@dataclass(frozen=True)
class ThrustCurve:
    """Piecewise-linear thrust history."""

    points: tuple[tuple[float, float], ...]

    def __post_init__(self) -> None:
        if len(self.points) < 2:
            raise ValueError("ThrustCurve requires at least two points.")
        previous_time = self.points[0][0]
        for time_s, thrust_n in self.points:
            if time_s < previous_time:
                raise ValueError("ThrustCurve times must be monotonic.")
            if thrust_n < 0.0:
                raise ValueError("Thrust cannot be negative.")
            previous_time = time_s

    def thrust_n(self, time_s: float) -> float:
        if time_s <= self.points[0][0]:
            return self.points[0][1]
        for (t0, thrust0), (t1, thrust1) in zip(self.points, self.points[1:]):
            if time_s <= t1:
                if t1 == t0:
                    return thrust1
                fraction = (time_s - t0) / (t1 - t0)
                return lerp(thrust0, thrust1, fraction)
        return self.points[-1][1]

    def impulse_n_s(self, start_s: float, end_s: float, steps: int = 400) -> float:
        if end_s <= start_s:
            return 0.0
        dt = (end_s - start_s) / steps
        total = 0.0
        previous = self.thrust_n(start_s)
        for index in range(1, steps + 1):
            current_time = start_s + index * dt
            current = self.thrust_n(current_time)
            total += 0.5 * (previous + current) * dt
            previous = current
        return total


@dataclass(frozen=True)
class MassPropertySchedule:
    """Propellant depletion model for vehicle mass, CM, and inertia."""

    initial_mass_kg: float
    dry_mass_kg: float
    initial_inertia_kg_m2: Vector
    dry_inertia_kg_m2: Vector
    initial_center_of_mass_body_m: Vector
    dry_center_of_mass_body_m: Vector
    isp_s: float
    thrust_curve: ThrustCurve
    gravity_mps2: float = 9.80665

    def __post_init__(self) -> None:
        if self.initial_mass_kg <= self.dry_mass_kg:
            raise ValueError("initial_mass_kg must exceed dry_mass_kg.")
        if self.dry_mass_kg <= 0.0:
            raise ValueError("dry_mass_kg must be positive.")
        if self.isp_s <= 0.0:
            raise ValueError("isp_s must be positive.")
        if any(value <= 0.0 for value in self.initial_inertia_kg_m2 + self.dry_inertia_kg_m2):
            raise ValueError("Inertia values must be positive.")

    @property
    def initial_propellant_mass_kg(self) -> float:
        return self.initial_mass_kg - self.dry_mass_kg

    def propellant_used_kg(self, time_s: float) -> float:
        impulse = self.thrust_curve.impulse_n_s(0.0, max(0.0, time_s))
        return min(self.initial_propellant_mass_kg, impulse / (self.isp_s * self.gravity_mps2))

    def remaining_propellant_fraction(self, time_s: float) -> float:
        used = self.propellant_used_kg(time_s)
        return max(0.0, 1.0 - used / self.initial_propellant_mass_kg)

    def mass_kg(self, time_s: float) -> float:
        return self.initial_mass_kg - self.propellant_used_kg(time_s)

    def inertia_kg_m2(self, time_s: float) -> Vector:
        fraction = self.remaining_propellant_fraction(time_s)
        return lerp_vec(self.dry_inertia_kg_m2, self.initial_inertia_kg_m2, fraction)

    def center_of_mass_body_m(self, time_s: float) -> Vector:
        fraction = self.remaining_propellant_fraction(time_s)
        return lerp_vec(self.dry_center_of_mass_body_m, self.initial_center_of_mass_body_m, fraction)


@dataclass(frozen=True)
class TimeVaryingRocket:
    """Generate instantaneous RocketParams from a mass-property schedule."""

    base_rocket: RocketParams
    mass_properties: MassPropertySchedule

    def at(self, time_s: float) -> RocketParams:
        return RocketParams(
            mass_kg=self.mass_properties.mass_kg(time_s),
            inertia_kg_m2=self.mass_properties.inertia_kg_m2(time_s),
            thrust_n=self.mass_properties.thrust_curve.thrust_n(time_s),
            reference_area_m2=self.base_rocket.reference_area_m2,
            drag_coefficient=self.base_rocket.drag_coefficient,
            normal_force_coefficient_per_rad=self.base_rocket.normal_force_coefficient_per_rad,
            center_of_mass_body_m=self.mass_properties.center_of_mass_body_m(time_s),
            center_of_pressure_body_m=self.base_rocket.center_of_pressure_body_m,
            thrust_offset_body_m=self.base_rocket.thrust_offset_body_m,
            thrust_direction_body=self.base_rocket.thrust_direction_body,
        )
