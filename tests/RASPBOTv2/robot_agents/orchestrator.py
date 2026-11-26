import time
from collections.abc import Mapping
from typing import Any

from langchain_core.tools import BaseTool


def create_orchestrator(
    robot_tools: Mapping[str, BaseTool],
    *,
    max_cycles: int,
    max_duration_seconds: float,
    min_photo_interval_seconds: float,
):
    async def orchestrator(state: dict[str, Any]) -> dict[str, Any]:
        next_agent = state["next_agent"]
        if next_agent == "photo":
            if state["status"] != "RUNNING":
                return {"next_agent": "end"}
            if state["cycles"] >= max_cycles:
                return {
                    "status": "CYCLE_LIMIT",
                    "reason": "Maximum exploration cycle limit reached.",
                    "next_agent": "end",
                }
            now = time.monotonic()
            if now - state["started_at"] >= max_duration_seconds:
                return {
                    "status": "TIME_LIMIT",
                    "reason": "Maximum exploration duration reached.",
                    "next_agent": "end",
                }

            updates: dict[str, Any] = {}
            last_photo_at = state["last_photo_at"]
            if (
                last_photo_at is None
                or now - last_photo_at >= min_photo_interval_seconds
            ):
                photo_path = await robot_tools["take_photo"].ainvoke({})
                updates["photos"] = [*state["photos"], photo_path]
                updates["last_photo_at"] = time.monotonic()
            updates["photo_checked"] = True
            updates["next_agent"] = "advancer"
            return updates

        if next_agent in ("explorer", "advancer"):
            if state["cycles"] >= max_cycles:
                return {
                    "status": "CYCLE_LIMIT",
                    "reason": "Maximum exploration cycle limit reached.",
                    "next_agent": "end",
                }
            if time.monotonic() - state["started_at"] >= max_duration_seconds:
                return {
                    "status": "TIME_LIMIT",
                    "reason": "Maximum exploration duration reached.",
                    "next_agent": "end",
                }
        return {}

    return orchestrator
