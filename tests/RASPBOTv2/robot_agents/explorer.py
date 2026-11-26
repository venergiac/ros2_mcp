import math
from collections.abc import Awaitable, Callable, Mapping
from typing import Any

from langchain_core.tools import BaseTool


ROTATION_GROUPS_DEGREES = (
    (0, -25, 50),
    (50, -100),
    (100,),
)


def rotation_label(degrees: int) -> str:
    return f"Z={degrees:+d}°"


async def align_to_selected_direction(
    current_heading_degrees: int,
    selected_angle_degrees: int | None,
    drive_robot: Callable[..., Awaitable[None]],
    *,
    turn_speed_rad_s: float,
    rotation_duration_seconds: Callable[[float], float],
) -> int:
    if selected_angle_degrees is None:
        raise ValueError("Cannot align without a selected direction.")
    delta = (selected_angle_degrees + 180) % 360 - 180
    if delta:
        target_heading = (current_heading_degrees + delta) % 360
        print(
            f"Controller: aligning to selected direction "
            f"({current_heading_degrees}° -> {target_heading}°, turn {delta:+d}°)."
        )
        await drive_robot(
            angular_z=math.copysign(turn_speed_rad_s, delta),
            duration=rotation_duration_seconds(delta),
        )
    return (current_heading_degrees + delta) % 360


def create_explorer(
    robot_tools: Mapping[str, BaseTool],
    drive_robot: Callable[..., Awaitable[None]],
    *,
    turn_speed_rad_s: float,
    rotation_duration_seconds: Callable[[float], float],
    minimum_clearance_cm: float,
):
    """Scan directions group by group and return the clearest safe direction."""

    async def rotate(current_heading: int, degrees: int) -> int:
        if degrees:
            await drive_robot(
                angular_z=math.copysign(turn_speed_rad_s, degrees),
                duration=rotation_duration_seconds(degrees),
            )
        return (current_heading + degrees) % 360

    async def explorer(state: dict[str, Any]) -> dict[str, Any]:
        readings: list[dict[str, Any]] = []
        heading = state["heading_degrees"]
        initial_heading = heading
        offset_degrees = 0
        selected_angle: int | None = None

        for group_index, group in enumerate(ROTATION_GROUPS_DEGREES, start=1):
            group_readings: list[tuple[int, float]] = []
            for rotation_degrees in group:
                heading = await rotate(heading, rotation_degrees)
                offset_degrees += rotation_degrees
                distance_cm = float(
                    await robot_tools["read_proximity"].ainvoke({})
                )
                direction = rotation_label(offset_degrees)
                readings.append(
                    {
                        "group": group_index,
                        "direction": direction,
                        "angle_degrees": offset_degrees,
                        "distance_cm": distance_cm,
                    }
                )
                group_readings.append((offset_degrees, distance_cm))
                print(
                    f"Explorer: group {group_index}, direction {direction}, "
                    f"clearance {distance_cm:.1f} cm"
                )

            safe_readings = [
                reading
                for reading in group_readings
                if reading[1] > minimum_clearance_cm
            ]
            if safe_readings:
                selected_angle = max(safe_readings, key=lambda item: item[1])[0]
                break

        selected_direction = (
            rotation_label(selected_angle) if selected_angle is not None else None
        )
        selected_clearance = None
        if selected_angle is not None:
            selected_clearance = max(
                distance
                for angle, distance in group_readings
                if angle == selected_angle
            )

        return_delta = (initial_heading - heading + 180) % 360 - 180
        if return_delta:
            print(
                "Explorer: restoring starting direction "
                f"(turn {return_delta:+d}°)."
            )
        heading = await rotate(heading, return_delta)

        distance = (
            selected_clearance
            if selected_clearance is not None
            else readings[-1]["distance_cm"]
            if readings
            else None
        )
        print(
            "Explorer decision: "
            f"{selected_direction or 'no safe direction'}"
            + (
                f" ({selected_clearance:.1f} cm)"
                if selected_clearance is not None
                else ""
            )
        )
        result = {
            "last_agent": "explorer",
            "heading_degrees": heading,
            "readings": readings,
            "selected_direction": selected_direction,
            "selected_angle_degrees": selected_angle,
            "selected_clearance_cm": selected_clearance,
            "previous_distance": state.get("distance"),
            "distance": distance,
            "scans": state["scans"] + 1,
        }
        agent_results = dict(state.get("agent_results", {}))
        agent_results["explorer"] = {
            "readings": readings,
            "selected_direction": selected_direction,
            "selected_clearance_cm": selected_clearance,
        }
        result["agent_results"] = agent_results
        return result

    return explorer
