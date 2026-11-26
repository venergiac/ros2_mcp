from collections.abc import Awaitable, Callable, Mapping
from typing import Any

from langchain_core.tools import BaseTool


def create_advancer(
    robot_tools: Mapping[str, BaseTool],
    drive_robot: Callable[..., Awaitable[None]],
    *,
    minimum_distance_cm: float,
    minimum_forward_pulse_seconds: float,
    maximum_forward_pulse_seconds: float,
    maximum_forward_steps: int,
    forward_speed_mps: float,
    forward_pulse_seconds: Callable[[float], float],
    forward_steps_for_clearance: Callable[[float], int],
):
    """Create the node that prechecks the route, then advances in safe pulses."""

    async def advancer(state: dict[str, Any]) -> dict[str, Any]:
        if not state["photo_checked"]:
            # Only request a photo after confirming that forward motion is safe.
            distance = float(
                await robot_tools["read_proximity"].ainvoke({})
            )
            if (
                distance <= minimum_distance_cm
                or forward_steps_for_clearance(distance) == 0
                or forward_pulse_seconds(distance) < minimum_forward_pulse_seconds
            ):
                return {
                    "next_agent": "explorer",
                    "cycles": state["cycles"] + 1,
                }
            return {"next_agent": "photo"}

        moves = 0
        for step_index in range(maximum_forward_steps):
            # Recheck clearance before every pulse and stop as soon as space shrinks.
            distance = float(
                await robot_tools["read_proximity"].ainvoke({})
            )
            if step_index >= forward_steps_for_clearance(distance):
                break

            duration = forward_pulse_seconds(distance)
            if duration < minimum_forward_pulse_seconds:
                break

            await drive_robot(
                linear_x=forward_speed_mps,
                duration=min(duration, maximum_forward_pulse_seconds),
            )
            moves += 1

        return {
            "next_agent": "explorer",
            "cycles": state["cycles"] + 1,
            "moves": state["moves"] + moves,
            "photo_checked": False,
        }

    return advancer
