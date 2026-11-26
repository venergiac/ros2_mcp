import math
from collections.abc import Awaitable, Callable, Mapping
from typing import Any

from langchain_core.tools import BaseTool

# Each group contains incremental yaw changes; later groups are tried only if
# the current group has no direction with sufficient clearance.
ROTATION_GROUPS_DEGREES = (
    (0, -25, 50),
    (50, -100),
    (100,),
)


def rotation_label(degrees: int) -> str:
    return f"Z={degrees:+d}°"


def select_direction(
    readings: Mapping[str, float],
    minimum_distance_cm: float,
) -> int | None:
    """Return the clearest safe direction, or None when every reading is blocked."""
    clear_readings = [
        (int(label.removeprefix("Z=").removesuffix("°")), distance)
        for label, distance in readings.items()
        if distance > minimum_distance_cm
    ]
    best_reading = max(clear_readings, key=lambda reading: reading[1], default=None)
    return best_reading[0] if best_reading is not None else None


def create_explorer(
    robot_tools: Mapping[str, BaseTool],
    drive_robot: Callable[..., Awaitable[None]],
    *,
    initial_turn_speed_rad_s: float,
    rotation_duration_seconds: Callable[[float], float],
    minimum_distance_cm: float,
):
    """Create the explorer node that scans directions and selects a safe heading."""

    async def turn_to_heading(current_heading: int, target_heading: int) -> int:
        delta = (target_heading - current_heading + 180) % 360 - 180
        if delta:
            await drive_robot(
                angular_z=math.copysign(initial_turn_speed_rad_s, delta),
                duration=rotation_duration_seconds(delta),
            )
        return target_heading % 360

    async def rotate_by_degrees(current_heading: int, degrees: int) -> int:
        if degrees:
            await drive_robot(
                angular_z=math.copysign(initial_turn_speed_rad_s, degrees),
                duration=rotation_duration_seconds(degrees),
            )
        return (current_heading + degrees) % 360

    async def explorer(state: dict[str, Any]) -> dict[str, Any]:
        """Measure each direction in order and choose the clearest safe one."""
        readings: dict[str, float] = {}
        heading = state["heading_degrees"]
        initial_heading = heading
        heading_offset_degrees = 0
        selected_degrees = None
        for group in ROTATION_GROUPS_DEGREES:
            group_readings: dict[str, float] = {}
            for rotation_degrees in group:
                heading_offset_degrees += rotation_degrees
                heading = await rotate_by_degrees(heading, rotation_degrees)
                distance = float(
                    await robot_tools["read_proximity"].ainvoke({})
                )
                label = rotation_label(heading_offset_degrees)
                readings[label] = distance
                group_readings[label] = distance
                print(
                    f"Incremental rotation {rotation_degrees:+d}° "
                    f"(heading {label}): measured distance {distance:.1f} cm"
                )
            selected_degrees = select_direction(
                group_readings,
                minimum_distance_cm,
            )
            if selected_degrees is not None:
                break

        if selected_degrees is None:
            return {
                "status": "BLOCKED",
                "reason": "No tested direction has sufficient clearance.",
                "next_agent": "end",
                "heading_degrees": heading,
                "selected_direction": None,
                "selected_angle": None,
                "selected_clearance_cm": None,
                "readings": readings,
                "scans": state["scans"] + 1,
            }

        heading = await turn_to_heading(
            heading,
            (initial_heading + selected_degrees) % 360,
        )
        selected_direction = rotation_label(selected_degrees)
        return {
            "next_agent": "advancer",
            "heading_degrees": heading,
            "selected_direction": selected_direction,
            "selected_angle": selected_degrees,
            "selected_clearance_cm": readings[selected_direction],
            "readings": readings,
            "scans": state["scans"] + 1,
        }

    return explorer
