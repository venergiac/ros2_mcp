import argparse
import asyncio
import json
import math
import os
import time
from collections.abc import Awaitable, Callable
from typing import Any, Literal, TypedDict

from langchain_core.tools import BaseTool
from langchain_mcp_adapters.client import MultiServerMCPClient
from langgraph.graph import END, StateGraph

from robot_agents.advancer import create_advancer
from robot_agents.explorer import align_to_selected_direction, create_explorer
from robot_agents.orchestrator import (
    advance_distance_for_clearance,
    create_orchestrator,
)
from robot_agents.photographer import create_photographer
from robot_tools import create_robot_tools
from sensors import decode_tool_result
from sensors.image import CAMERA_TOPIC, resolve_camera_topic


MIN_CLEARANCE_CM = 25.0
FORWARD_SAFETY_MARGIN_CM = 2.0
FORWARD_STEP_FRACTION = 0.5
MAX_DURATION_SECONDS = 15 * 60.0
MAX_CYCLES = 1800
MIN_PHOTO_INTERVAL_SECONDS = 60.0

FORWARD_SPEED_MPS = 0.05
FORWARD_PULSE_SECONDS = 2.0
MIN_FORWARD_PULSE_SECONDS = 0.1
MAX_ADVANCE_DISTANCE_CM = FORWARD_SPEED_MPS * FORWARD_PULSE_SECONDS * 100

TURN_ANGULAR_SPEED_RAD_S = 0.25
MAX_ANGULAR_SPEED_RAD_S = 1.0
TURN_RATE_CALIBRATION_FACTOR = 225.0 / 45.0
CALIBRATED_TURN_RATE_RAD_S = (
    TURN_ANGULAR_SPEED_RAD_S * TURN_RATE_CALIBRATION_FACTOR
)
STOP_PULSE_SECONDS = 0.2
MIN_ADVANCE_DISTANCE_CM = (
    MIN_CLEARANCE_CM
    + FORWARD_SAFETY_MARGIN_CM
    + 2 * FORWARD_SPEED_MPS * MIN_FORWARD_PULSE_SECONDS * 100
)


class AgentState(TypedDict):
    next_action: Literal["explorer", "advancer", "photographer", "end"]
    last_agent: str
    status: str
    reason: str
    started_at: float
    cycles: int
    scans: int
    heading_degrees: int
    selected_direction: str | None
    selected_angle_degrees: int | None
    selected_clearance_cm: float | None
    readings: list[dict[str, Any]]
    distance: float | None
    previous_distance: float | None
    advance_distance_cm: float
    last_advance_distance_cm: float | None
    last_advance_duration_seconds: float | None
    last_photo_path: str | None
    last_photo_at: float | None
    photos: list[str]
    agent_results: dict[str, dict[str, Any]]


def rotation_duration_seconds(angle_degrees: float) -> float:
    return abs(math.radians(angle_degrees) / CALIBRATED_TURN_RATE_RAD_S)


def create_driver(
    driver_tool: BaseTool,
) -> Callable[..., Awaitable[None]]:
    async def drive_robot(
        linear_x: float = 0.0,
        angular_z: float = 0.0,
        duration: float = STOP_PULSE_SECONDS,
    ) -> None:
        if not all(
            math.isfinite(value) for value in (linear_x, angular_z, duration)
        ):
            raise ValueError("Motion parameters must be finite.")
        if abs(linear_x) > FORWARD_SPEED_MPS:
            raise ValueError("Linear velocity exceeds the configured limit.")
        if abs(angular_z) > MAX_ANGULAR_SPEED_RAD_S:
            raise ValueError("Angular velocity exceeds the configured limit.")
        if linear_x and angular_z:
            raise ValueError("Simultaneous translation and rotation are not allowed.")
        if linear_x and not (
            MIN_FORWARD_PULSE_SECONDS <= duration <= FORWARD_PULSE_SECONDS
        ):
            raise ValueError("Forward duration is outside the configured limits.")
        if (
            angular_z
            and abs(angular_z * TURN_RATE_CALIBRATION_FACTOR * duration) > math.pi
        ):
            raise ValueError("Rotation exceeds the 180-degree limit.")
        if not linear_x and not angular_z and duration > STOP_PULSE_SECONDS:
            raise ValueError("Stop command duration exceeds the configured limit.")

        command = {
            "topic_name": "/cmd_vel",
            "linear_x": linear_x,
            "linear_y": 0.0,
            "linear_z": 0.0,
            "angular_x": 0.0,
            "angular_y": 0.0,
            "angular_z": angular_z,
            "duration_sec": duration,
        }
        print(
            "ROS 2 /cmd_vel vector: "
            f"linear=({linear_x!r}, 0.0, 0.0), "
            f"angular=(0.0, 0.0, {angular_z!r}), "
            f"duration={duration!r}s"
        )
        result = decode_tool_result(await driver_tool.ainvoke(command))
        if not isinstance(result, dict):
            raise TypeError("drive_robot returned an unexpected response.")
        if "error" in result:
            raise RuntimeError(f"drive_robot failed: {result['error']}")
        if result.get("status") != "completed":
            raise RuntimeError(
                f"Unexpected drive_robot status: {result.get('status')!r}."
            )

    return drive_robot


def create_agent_graph(
    robot_tools: dict[str, BaseTool],
    drive_robot: Callable[..., Awaitable[None]],
    *,
    orchestrator_kind: Literal["deterministic", "llm"],
):
    orchestrator_options = {
        "max_cycles": MAX_CYCLES,
        "max_duration_seconds": MAX_DURATION_SECONDS,
        "min_photo_interval_seconds": MIN_PHOTO_INTERVAL_SECONDS,
        "minimum_clearance_cm": MIN_ADVANCE_DISTANCE_CM,
        "obstacle_clearance_cm": MIN_CLEARANCE_CM,
        "obstacle_safety_margin_cm": FORWARD_SAFETY_MARGIN_CM,
        "advance_fraction": FORWARD_STEP_FRACTION,
        "maximum_advance_distance_cm": MAX_ADVANCE_DISTANCE_CM,
        "minimum_advance_duration_seconds": MIN_FORWARD_PULSE_SECONDS,
        "forward_speed_mps": FORWARD_SPEED_MPS,
    }
    if orchestrator_kind == "deterministic":
        orchestrator = create_orchestrator(**orchestrator_options)
    elif orchestrator_kind == "llm":
        provider = os.environ.get("MODEL_PROVIDER", "groq").strip().lower()
        if provider != "groq":
            raise ValueError(
                f"Unsupported MODEL_PROVIDER={provider!r}; expected 'groq'."
            )
        if not os.environ.get("GROQ_API_KEY"):
            raise RuntimeError("GROQ_API_KEY is required for the LLM orchestrator.")
        from robot_agents.orchestrator_llm import create_orchestrator_llm

        orchestrator = create_orchestrator_llm(
            **orchestrator_options,
            model=os.environ.get("MODEL_NAME", "qwen/qwen3.8-27b"),
        )
    else:
        raise ValueError(f"Unknown orchestrator: {orchestrator_kind!r}")

    agents = create_subagents(robot_tools, drive_robot)

    def route_from_orchestrator(state: AgentState) -> str:
        return state["next_action"]

    async def advancer_after_alignment(state: AgentState) -> dict[str, Any]:
        try:
            heading = await align_to_selected_direction(
                state["heading_degrees"],
                state["selected_angle_degrees"],
                drive_robot,
                turn_speed_rad_s=TURN_ANGULAR_SPEED_RAD_S,
                rotation_duration_seconds=rotation_duration_seconds,
            )
            result = await agents["advancer"](state)
            return {**result, "heading_degrees": heading}
        finally:
            await drive_robot(
                linear_x=0.0,
                angular_z=0.0,
                duration=STOP_PULSE_SECONDS,
            )

    graph = StateGraph(AgentState)
    graph.add_node("orchestrator", orchestrator)
    graph.add_node("explorer", agents["explorer"])
    graph.add_node("advancer", advancer_after_alignment)
    graph.add_node("photographer", agents["photographer"])
    graph.set_entry_point("orchestrator")
    graph.add_conditional_edges(
        "orchestrator",
        route_from_orchestrator,
        {
            "explorer": "explorer",
            "advancer": "advancer",
            "photographer": "photographer",
            "end": END,
        },
    )
    for agent_name in ("explorer", "advancer", "photographer"):
        graph.add_edge(agent_name, "orchestrator")
    return graph.compile()


def create_subagents(
    robot_tools: dict[str, BaseTool],
    drive_robot: Callable[..., Awaitable[None]],
) -> dict[str, Any]:
    return {
        "explorer": create_explorer(
            robot_tools,
            drive_robot,
            turn_speed_rad_s=TURN_ANGULAR_SPEED_RAD_S,
            rotation_duration_seconds=rotation_duration_seconds,
            minimum_clearance_cm=MIN_ADVANCE_DISTANCE_CM,
        ),
        "advancer": create_advancer(
            drive_robot,
            forward_speed_mps=FORWARD_SPEED_MPS,
            maximum_advance_distance_cm=MAX_ADVANCE_DISTANCE_CM,
        ),
        "photographer": create_photographer(robot_tools),
    }


async def run_exploration(
    tools: list[BaseTool],
    camera_topic: str,
    *,
    orchestrator_kind: Literal["deterministic", "llm"] = "deterministic",
    stop_on_exit: bool = True,
) -> AgentState:
    tools_by_name = {tool.name: tool for tool in tools}
    driver_tool = tools_by_name.get("drive_robot")
    if driver_tool is None:
        raise RuntimeError("Missing MCP tool: drive_robot")
    drive_robot = create_driver(driver_tool)
    required = {"topic_echo", "drive_robot", "get_image"}
    missing = required - tools_by_name.keys()
    try:
        if missing:
            raise RuntimeError(f"Missing MCP tools: {sorted(missing)}")
        robot_tools = create_robot_tools(
            tools_by_name["topic_echo"],
            tools_by_name["get_image"],
            camera_topic,
        )
        graph = create_agent_graph(
            robot_tools,
            drive_robot,
            orchestrator_kind=orchestrator_kind,
        )
        initial_state: AgentState = {
            "next_action": "explorer",
            "last_agent": "start",
            "status": "RUNNING",
            "reason": "",
            "started_at": time.monotonic(),
            "cycles": 0,
            "scans": 0,
            "heading_degrees": 0,
            "selected_direction": None,
            "selected_angle_degrees": None,
            "selected_clearance_cm": None,
            "readings": [],
            "distance": None,
            "previous_distance": None,
            "advance_distance_cm": 0.0,
            "last_advance_distance_cm": None,
            "last_advance_duration_seconds": None,
            "last_photo_path": None,
            "last_photo_at": None,
            "photos": [],
            "agent_results": {},
        }
        return await graph.ainvoke(
            initial_state,
            config={"recursion_limit": MAX_CYCLES * 4 + 8},
        )
    finally:
        if stop_on_exit:
            print("Shutdown: sending zero velocity on every axis.")
            await drive_robot(
                linear_x=0.0,
                angular_z=0.0,
                duration=STOP_PULSE_SECONDS,
            )


async def main() -> None:
    parser = argparse.ArgumentParser(description="Explore with specialized robot agents.")
    parser.add_argument(
        "--orchestrator",
        choices=("deterministic", "llm"),
        default=os.environ.get("ROBOT_ORCHESTRATOR", "deterministic"),
        help="Choose the deterministic or Groq-powered LLM orchestrator.",
    )
    args = parser.parse_args()

    mcp_url = os.environ.get("ROS2_MCP_URL", "http://localhost:8000/sse")
    client = MultiServerMCPClient(
        {"ros2_mcp": {"transport": "sse", "url": mcp_url}}
    )
    tools = await client.get_tools()
    tools_by_name = {tool.name: tool for tool in tools}
    driver_tool = tools_by_name.get("drive_robot")
    if driver_tool is None:
        raise RuntimeError("Missing MCP tool: drive_robot")
    drive_robot = create_driver(driver_tool)
    required = {"list_topics", "get_image", "topic_echo", "drive_robot"}
    missing = required - tools_by_name.keys()
    try:
        if missing:
            raise RuntimeError(f"Missing MCP tools: {sorted(missing)}")
        camera_topic = await resolve_camera_topic(
            tools_by_name["list_topics"], CAMERA_TOPIC
        )
        confirmation = input(
            f"Orchestrator: {args.orchestrator}. Supervise the robot and clear "
            "the area, then type GO to start (maximum 15 minutes): "
        ).strip().upper()
        if confirmation != "GO":
            return

        result = await run_exploration(
            tools,
            camera_topic,
            orchestrator_kind=args.orchestrator,
            stop_on_exit=False,
        )
        print(f"Exploration ended: {result['status']} - {result['reason']}")
    finally:
        print("Shutdown: sending zero velocity on every axis.")
        await drive_robot(
            linear_x=0.0,
            angular_z=0.0,
            duration=STOP_PULSE_SECONDS,
        )


if __name__ == "__main__":
    asyncio.run(main())
