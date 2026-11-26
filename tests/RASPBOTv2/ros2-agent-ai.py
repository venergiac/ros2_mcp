import asyncio
import math
import os
import time
from collections.abc import Awaitable, Callable
from typing import Literal, TypedDict

from langchain_core.tools import BaseTool
from langchain_mcp_adapters.client import MultiServerMCPClient
from langgraph.graph import END, StateGraph

from robot_agents.advancer import create_advancer
from robot_agents.explorer import create_explorer
from robot_agents.orchestrator import create_orchestrator
from robot_tools import create_robot_tools
from sensors.image import (
    CAMERA_TOPIC,
    resolve_camera_topic,
)

# Minimum measured obstacle distance required before advancing, in centimeters.
MIN_CLEARANCE_CM = 25.0
# Extra space kept between the robot and an obstacle.
FORWARD_SAFETY_MARGIN_CM = 2.0
# Hard limits on total runtime and navigation cycles.
MAX_DURATION_SECONDS = 15 * 60.0
MAX_CYCLES = 1800
# Minimum time between consecutive camera captures.
MIN_PHOTO_INTERVAL_SECONDS = 60.0

# Constant forward velocity and maximum duration of a single forward pulse.
FORWARD_SPEED_MPS = 0.05
FORWARD_PULSE_SECONDS = 2
# Fraction of available clearance used for each forward step.
FORWARD_STEP_FRACTION = 0.5
# Shortest permitted forward pulse and maximum pulses in one advance phase.
MIN_FORWARD_PULSE_SECONDS = 0.1
MAX_FORWARD_STEPS_PER_ADVANCE = 8
# ROS angular.z command speed and its maximum allowed absolute value, in rad/s.
TURN_ANGULAR_SPEED_RAD_S = 0.25
MAX_ANGULAR_SPEED_RAD_S = 1.0
# Empirical correction: a requested 45-degree turn produced about 225 degrees.
TURN_RATE_CALIBRATION_FACTOR = 225.0 / 45.0
# Effective turn rate used to convert a requested angle into command duration.
CALIBRATED_TURN_RATE_RAD_S = (
    TURN_ANGULAR_SPEED_RAD_S * TURN_RATE_CALIBRATION_FACTOR
)
# Duration of a zero-velocity command used to stop the robot.
STOP_PULSE_SECONDS = 0.2
# Sensor distance needed to allow at least one minimum-length forward pulse.
MIN_ADVANCE_DISTANCE_CM = (
    MIN_CLEARANCE_CM
    + FORWARD_SAFETY_MARGIN_CM
    + 2 * FORWARD_SPEED_MPS * MIN_FORWARD_PULSE_SECONDS * 100
)
# Maximum distance, in centimeters, covered by one full-speed forward pulse.
MAX_FORWARD_STEP_DISTANCE_CM = (
    FORWARD_SPEED_MPS * FORWARD_PULSE_SECONDS * 100
)


class AgentState(TypedDict):
    next_agent: Literal["explorer", "advancer", "photo", "end"]
    status: str
    reason: str
    started_at: float
    last_photo_at: float | None
    photo_checked: bool
    cycles: int
    moves: int
    scans: int
    heading_degrees: int
    selected_direction: str | None
    selected_angle: int | None
    selected_clearance_cm: float | None
    readings: dict[str, float]
    photos: list[str]


def forward_pulse_seconds(distance_cm: float) -> float:
    safe_step_cm = max(
        0.0,
        (distance_cm - MIN_CLEARANCE_CM - FORWARD_SAFETY_MARGIN_CM)
        * FORWARD_STEP_FRACTION,
    )
    duration = safe_step_cm / 100.0 / FORWARD_SPEED_MPS
    return min(FORWARD_PULSE_SECONDS, duration)


def forward_steps_for_clearance(distance_cm: float) -> int:
    safe_travel_cm = max(
        0.0,
        (distance_cm - MIN_ADVANCE_DISTANCE_CM) * FORWARD_STEP_FRACTION,
    )
    if safe_travel_cm == 0:
        return 0
    return min(
        MAX_FORWARD_STEPS_PER_ADVANCE,
        math.ceil(safe_travel_cm / MAX_FORWARD_STEP_DISTANCE_CM),
    )


def rotation_duration_seconds(angle_degrees: float) -> float:
    return abs(math.radians(angle_degrees) / CALIBRATED_TURN_RATE_RAD_S)


def create_agent_graph(
    robot_tools: dict[str, BaseTool],
    drive_robot: Callable[..., Awaitable[None]],
):
    orchestrator = create_orchestrator(
        robot_tools,
        max_cycles=MAX_CYCLES,
        max_duration_seconds=MAX_DURATION_SECONDS,
        min_photo_interval_seconds=MIN_PHOTO_INTERVAL_SECONDS,
    )
    explorer = create_explorer(
        robot_tools,
        drive_robot,
        initial_turn_speed_rad_s=TURN_ANGULAR_SPEED_RAD_S,
        rotation_duration_seconds=rotation_duration_seconds,
        minimum_distance_cm=MIN_ADVANCE_DISTANCE_CM,
    )
    advancer = create_advancer(
        robot_tools,
        drive_robot,
        minimum_distance_cm=MIN_ADVANCE_DISTANCE_CM,
        minimum_forward_pulse_seconds=MIN_FORWARD_PULSE_SECONDS,
        maximum_forward_pulse_seconds=FORWARD_PULSE_SECONDS,
        maximum_forward_steps=MAX_FORWARD_STEPS_PER_ADVANCE,
        forward_speed_mps=FORWARD_SPEED_MPS,
        forward_pulse_seconds=forward_pulse_seconds,
        forward_steps_for_clearance=forward_steps_for_clearance,
    )

    def route_from_orchestrator(state: AgentState) -> str:
        return state["next_agent"]

    graph = StateGraph(AgentState)
    graph.add_node("orchestrator", orchestrator)
    graph.add_node("explorer", explorer)
    graph.add_node("advancer", advancer)
    graph.set_entry_point("orchestrator")
    graph.add_conditional_edges(
        "orchestrator",
        route_from_orchestrator,
        {
            "explorer": "explorer",
            "advancer": "advancer",
            "end": END,
        },
    )
    graph.add_edge("explorer", "orchestrator")
    graph.add_edge("advancer", "orchestrator")
    return graph.compile()


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
        if linear_x < 0:
            raise ValueError("Reverse motion is disabled.")
        if linear_x > FORWARD_SPEED_MPS:
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
            f"linear=({command['linear_x']!r}, {command['linear_y']!r}, "
            f"{command['linear_z']!r}), "
            f"angular=({command['angular_x']!r}, {command['angular_y']!r}, "
            f"{command['angular_z']!r}), "
            f"duration={command['duration_sec']!r}s"
        )
        result = await driver_tool.ainvoke(command)
        from sensors import decode_tool_result

        result = decode_tool_result(result)
        if not isinstance(result, dict):
            raise TypeError("drive_robot returned an unexpected response.")
        if "error" in result:
            raise RuntimeError(f"drive_robot failed: {result['error']}")
        if result.get("status") != "completed":
            raise RuntimeError(
                f"Unexpected drive_robot status: {result.get('status')!r}."
            )

    return drive_robot


async def run_exploration(
    tools: list[BaseTool],
    camera_topic: str,
) -> AgentState:
    tools_by_name = {tool.name: tool for tool in tools}
    required = {"topic_echo", "drive_robot", "get_image"}
    missing = required - tools_by_name.keys()
    if missing:
        raise RuntimeError(f"Missing MCP tools: {sorted(missing)}")

    robot_tools = create_robot_tools(
        tools_by_name["topic_echo"],
        tools_by_name["get_image"],
        camera_topic,
    )
    drive_robot = create_driver(tools_by_name["drive_robot"])
    graph = create_agent_graph(robot_tools, drive_robot)
    initial_state: AgentState = {
        "next_agent": "explorer",
        "status": "RUNNING",
        "reason": "Exploration completed.",
        "started_at": time.monotonic(),
        "last_photo_at": None,
        "photo_checked": False,
        "cycles": 0,
        "moves": 0,
        "scans": 0,
        "heading_degrees": 0,
        "selected_direction": None,
        "selected_angle": None,
        "selected_clearance_cm": None,
        "readings": {},
        "photos": [],
    }

    try:
        result = await graph.ainvoke(
            initial_state,
            config={"recursion_limit": MAX_CYCLES * 4 + 8},
        )
        return result
    finally:
        await drive_robot()


async def main() -> None:
    mcp_url = os.environ.get("ROS2_MCP_URL", "http://localhost:8000/sse")
    client = MultiServerMCPClient(
        {"ros2_mcp": {"transport": "sse", "url": mcp_url}}
    )
    tools = await client.get_tools()
    tools_by_name = {tool.name: tool for tool in tools}
    required = {"list_topics", "get_image", "topic_echo", "drive_robot"}
    missing = required - tools_by_name.keys()
    if missing:
        raise RuntimeError(f"Missing MCP tools: {sorted(missing)}")

    camera_topic = await resolve_camera_topic(
        tools_by_name["list_topics"], CAMERA_TOPIC
    )
    confirmation = input(
        "Is the robot supervised and the area clear? "
        "Type GO to start (maximum 15 minutes): "
    ).strip().upper()
    if confirmation != "GO":
        return

    await run_exploration(tools, camera_topic)


if __name__ == "__main__":
    asyncio.run(main())
