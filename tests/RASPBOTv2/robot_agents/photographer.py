import time
from collections.abc import Mapping
from typing import Any

from langchain_core.tools import BaseTool


def create_photographer(robot_tools: Mapping[str, BaseTool]):
    """Capture and report a photo without making navigation decisions."""

    async def photographer(state: dict[str, Any]) -> dict[str, Any]:
        photo_path = str(await robot_tools["take_photo"].ainvoke({}))
        photos = [*state["photos"], photo_path]
        print(f"Photographer: saved {photo_path}")
        agent_results = dict(state.get("agent_results", {}))
        agent_results["photographer"] = {"photo_path": photo_path}
        return {
            "last_agent": "photographer",
            "last_photo_path": photo_path,
            "last_photo_at": time.monotonic(),
            "photos": photos,
            "agent_results": agent_results,
        }

    return photographer
