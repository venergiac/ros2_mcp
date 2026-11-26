import time
from typing import Any


def advance_distance_for_clearance(
    clearance_cm: float,
    *,
    obstacle_clearance_cm: float,
    obstacle_safety_margin_cm: float,
    advance_fraction: float,
    maximum_advance_distance_cm: float,
) -> float:
    available_cm = max(
        0.0,
        (clearance_cm - obstacle_clearance_cm - obstacle_safety_margin_cm)
        * advance_fraction,
    )
    return min(available_cm, maximum_advance_distance_cm)


def create_orchestrator(
    *,
    max_cycles: int,
    max_duration_seconds: float,
    min_photo_interval_seconds: float,
    minimum_clearance_cm: float,
    obstacle_clearance_cm: float,
    obstacle_safety_margin_cm: float,
    advance_fraction: float,
    maximum_advance_distance_cm: float,
    minimum_advance_duration_seconds: float,
    forward_speed_mps: float,
):
    """Route between specialized agents using clearance and their latest results."""

    async def orchestrator(state: dict[str, Any]) -> dict[str, Any]:
        now = time.monotonic()
        if state["cycles"] >= max_cycles:
            print("Orchestrator: cycle limit reached; stopping.")
            return {
                "status": "CYCLE_LIMIT",
                "reason": "Maximum navigation cycle limit reached.",
                "next_action": "end",
            }
        if now - state["started_at"] >= max_duration_seconds:
            print("Orchestrator: duration limit reached; stopping.")
            return {
                "status": "TIME_LIMIT",
                "reason": "Maximum navigation duration reached.",
                "next_action": "end",
            }

        last_agent = state["last_agent"]
        if last_agent == "start" or last_agent == "advancer":
            updates: dict[str, Any] = {"next_action": "explorer"}
            if last_agent == "advancer":
                updates.update(
                    {
                        "cycles": state["cycles"] + 1,
                        "advance_distance_cm": 0.0,
                    }
                )
            print(f"Orchestrator: {last_agent} -> explorer.")
            return updates

        if last_agent == "explorer":
            clearance_cm = state["selected_clearance_cm"]
            if (
                state["selected_direction"] is None
                or clearance_cm is None
                or clearance_cm <= minimum_clearance_cm
            ):
                print("Orchestrator: no direction has sufficient clearance; stopping.")
                return {
                    "status": "BLOCKED",
                    "reason": "Explorer found no direction with safe clearance.",
                    "next_action": "end",
                }

            advance_distance_cm = advance_distance_for_clearance(
                clearance_cm,
                obstacle_clearance_cm=obstacle_clearance_cm,
                obstacle_safety_margin_cm=obstacle_safety_margin_cm,
                advance_fraction=advance_fraction,
                maximum_advance_distance_cm=maximum_advance_distance_cm,
            )
            advance_duration = advance_distance_cm / (forward_speed_mps * 100)
            if advance_duration < minimum_advance_duration_seconds:
                print("Orchestrator: clearance is too small for a safe forward pulse.")
                return {
                    "status": "BLOCKED",
                    "reason": "Available clearance is too small for a safe advance.",
                    "next_action": "end",
                }

            updates = {"advance_distance_cm": advance_distance_cm}
            last_photo_at = state["last_photo_at"]
            if (
                last_photo_at is None
                or now - last_photo_at >= min_photo_interval_seconds
            ):
                updates["next_action"] = "photographer"
            else:
                updates["next_action"] = "advancer"
            print(
                f"Orchestrator: {state['selected_direction']} has "
                f"{clearance_cm:.1f} cm clearance; requesting "
                f"{advance_distance_cm:.1f} cm advance, then "
                f"{updates['next_action']}."
            )
            return updates

        if last_agent == "photographer":
            print("Orchestrator: photographer -> advancer.")
            return {"next_action": "advancer"}

        raise ValueError(f"Unknown last agent: {last_agent!r}")

    return orchestrator
