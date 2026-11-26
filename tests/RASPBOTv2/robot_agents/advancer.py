import math
from collections.abc import Awaitable, Callable
from typing import Any


def create_advancer(
    drive_robot: Callable[..., Awaitable[None]],
    *,
    forward_speed_mps: float,
    maximum_advance_distance_cm: float,
):
    """Move forward by exactly the distance requested in the graph state."""

    async def advancer(state: dict[str, Any]) -> dict[str, Any]:
        distance_cm = state["advance_distance_cm"]
        if not math.isfinite(distance_cm) or distance_cm <= 0:
            raise ValueError("Requested advance distance must be a positive finite value.")
        if distance_cm > maximum_advance_distance_cm:
            raise ValueError(
                "Requested advance distance exceeds the configured safe maximum."
            )

        duration = distance_cm / (forward_speed_mps * 100)
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError("Requested advance duration is invalid.")

        print(
            f"Advancer: moving forward {distance_cm:.1f} cm "
            f"at {forward_speed_mps:.3f} m/s for {duration:.2f} s."
        )
        await drive_robot(linear_x=forward_speed_mps, duration=duration)
        result = {
            "last_agent": "advancer",
            "last_advance_distance_cm": distance_cm,
            "last_advance_duration_seconds": duration,
        }
        agent_results = dict(state.get("agent_results", {}))
        agent_results["advancer"] = {
            "requested_distance_cm": distance_cm,
            "duration_seconds": duration,
        }
        result["agent_results"] = agent_results
        return result

    return advancer
