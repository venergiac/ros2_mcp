import asyncio
import math
import os
import time
from collections.abc import Awaitable, Callable
from typing import Literal, TypedDict

from langchain_core.tools import BaseTool
from langchain_mcp_adapters.client import MultiServerMCPClient
from langgraph.graph import END, StateGraph

from robot_tools import create_robot_tools
from sensors.image import (
    CAMERA_TOPIC,
    resolve_camera_topic,
)

MIN_CLEARANCE_CM = 25.0
FORWARD_SAFETY_MARGIN_CM = 2.0
MAX_DURATION_SECONDS = 15 * 60.0
MAX_CYCLES = 1800
MIN_PHOTO_INTERVAL_SECONDS = 60.0

FORWARD_SPEED_MPS = 0.05
FORWARD_PULSE_SECONDS = 0.5
FORWARD_STEP_FRACTION = 0.5
MIN_FORWARD_PULSE_SECONDS = 0.1
MAX_FORWARD_STEPS_PER_ADVANCE = 8
TURN_ANGULAR_SPEED_RAD_S = 0.25
MAX_ANGULAR_SPEED_RAD_S = 1.0
TURN_RATE_CALIBRATION_FACTOR = 225.0 / 45.0
CALIBRATED_TURN_RATE_RAD_S = (
    TURN_ANGULAR_SPEED_RAD_S * TURN_RATE_CALIBRATION_FACTOR
)
STOP_PULSE_SECONDS = 0.2
ROTATION_GROUPS_DEGREES = (
    (-25, 50),
    (50, -100),
    (100,),
)
MIN_ADVANCE_DISTANCE_CM = (
    MIN_CLEARANCE_CM
    + FORWARD_SAFETY_MARGIN_CM
    + 2 * FORWARD_SPEED_MPS * MIN_FORWARD_PULSE_SECONDS * 100
)
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


def rotation_label(degrees: int) -> str:
    return f"Z={degrees:+d}°"


def rotation_duration_seconds(angle_degrees: float) -> float:
    return abs(math.radians(angle_degrees) / CALIBRATED_TURN_RATE_RAD_S)


def select_direction(readings: dict[str, float]) -> int | None:
    clear_readings = [
        (int(label.removeprefix("Z=").removesuffix("°")), distance)
        for label, distance in readings.items()
        if distance > MIN_ADVANCE_DISTANCE_CM
    ]
    best_reading = max(clear_readings, key=lambda reading: reading[1], default=None)
    return best_reading[0] if best_reading is not None else None


def create_agent_graph(
    robot_tools: dict[str, BaseTool],
    drive_robot: Callable[..., Awaitable[None]],
):
    async def turn_to_heading(current_heading: int, target_heading: int) -> int:
        delta = (target_heading - current_heading + 180) % 360 - 180
        if delta:
            await drive_robot(
                angular_z=math.copysign(TURN_ANGULAR_SPEED_RAD_S, delta),
                duration=rotation_duration_seconds(delta),
            )
        return target_heading % 360

    async def rotate_by_degrees(current_heading: int, degrees: int) -> int:
        delta_degrees = degrees
        if delta_degrees:
            await drive_robot(
                angular_z=math.copysign(
                    TURN_ANGULAR_SPEED_RAD_S,
                    delta_degrees,
                ),
                duration=rotation_duration_seconds(delta_degrees),
            )
        return (current_heading + delta_degrees) % 360

    async def orchestrator(state: AgentState) -> dict:
        next_agent = state["next_agent"]
        if next_agent == "photo":
            if state["status"] != "RUNNING":
                return {"next_agent": "end"}
            if state["cycles"] >= MAX_CYCLES:
                return {
                    "status": "CYCLE_LIMIT",
                    "reason": "Limite massimo di cicli raggiunto.",
                    "next_agent": "end",
                }
            now = time.monotonic()
            if now - state["started_at"] >= MAX_DURATION_SECONDS:
                return {
                    "status": "TIME_LIMIT",
                    "reason": "Limite massimo di durata raggiunto.",
                    "next_agent": "end",
                }

            updates: dict = {}
            last_photo_at = state["last_photo_at"]
            if (
                last_photo_at is None
                or now - last_photo_at >= MIN_PHOTO_INTERVAL_SECONDS
            ):
                photo_path = await robot_tools["take_photo"].ainvoke({})
                updates["photos"] = [*state["photos"], photo_path]
                updates["last_photo_at"] = time.monotonic()
            updates["photo_checked"] = True
            updates["next_agent"] = "advancer"
            return updates

        if next_agent in ("explorer", "advancer"):
            if state["cycles"] >= MAX_CYCLES:
                return {
                    "status": "CYCLE_LIMIT",
                    "reason": "Limite massimo di cicli raggiunto.",
                    "next_agent": "end",
                }
            if time.monotonic() - state["started_at"] >= MAX_DURATION_SECONDS:
                return {
                    "status": "TIME_LIMIT",
                    "reason": "Limite massimo di durata raggiunto.",
                    "next_agent": "end",
                }
        return {}

    async def explorer(state: AgentState) -> dict:
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
                    f"Rotazione incrementale {rotation_degrees:+d}° "
                    f"(direzione {label}): distanza misurata {distance:.1f} cm"
                )
            selected_degrees = select_direction(group_readings)
            if selected_degrees is not None:
                break

        if selected_degrees is None:
            return {
                "status": "BLOCKED",
                "reason": "Nessuna delle otto direzioni ha spazio sufficiente.",
                "next_agent": "end",
                "heading_degrees": heading,
                "selected_direction": None,
                "selected_angle": None,
                "selected_clearance_cm": None,
                "readings": readings,
                "scans": state["scans"] + 1,
            }

        selected_angle = selected_degrees
        heading = await turn_to_heading(
            heading,
            (initial_heading + selected_angle) % 360,
        )
        selected_direction = rotation_label(selected_degrees)
        selected_clearance = readings[selected_direction]
        return {
            "next_agent": "advancer",
            "heading_degrees": heading,
            "selected_direction": selected_direction,
            "selected_angle": selected_angle,
            "selected_clearance_cm": selected_clearance,
            "readings": readings,
            "scans": state["scans"] + 1,
        }

    async def advancer(state: AgentState) -> dict:
        if not state["photo_checked"]:
            distance = float(
                await robot_tools["read_proximity"].ainvoke({})
            )
            if (
                distance <= MIN_ADVANCE_DISTANCE_CM
                or forward_steps_for_clearance(distance) == 0
                or forward_pulse_seconds(distance) < MIN_FORWARD_PULSE_SECONDS
            ):
                return {
                    "next_agent": "explorer",
                    "cycles": state["cycles"] + 1,
                }
            return {"next_agent": "photo"}

        moves = 0
        for step_index in range(MAX_FORWARD_STEPS_PER_ADVANCE):
            distance = float(
                await robot_tools["read_proximity"].ainvoke({})
            )
            if step_index >= forward_steps_for_clearance(distance):
                break

            duration = forward_pulse_seconds(distance)
            if duration < MIN_FORWARD_PULSE_SECONDS:
                break

            await drive_robot(
                linear_x=FORWARD_SPEED_MPS,
                duration=min(duration, FORWARD_PULSE_SECONDS),
            )
            moves += 1

        return {
            "next_agent": "explorer",
            "cycles": state["cycles"] + 1,
            "moves": state["moves"] + moves,
            "photo_checked": False,
        }

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
            raise ValueError("I parametri di movimento devono essere finiti.")
        if linear_x < 0:
            raise ValueError("La retromarcia non e' abilitata.")
        if linear_x > FORWARD_SPEED_MPS:
            raise ValueError("Velocità lineare oltre il limite configurato.")
        if abs(angular_z) > MAX_ANGULAR_SPEED_RAD_S:
            raise ValueError("Velocità angolare oltre il limite configurato.")
        if linear_x and angular_z:
            raise ValueError("Traslazione e rotazione simultanee non consentite.")
        if linear_x and not (
            MIN_FORWARD_PULSE_SECONDS <= duration <= FORWARD_PULSE_SECONDS
        ):
            raise ValueError("Durata dell'avanzamento fuori dai limiti configurati.")
        if (
            angular_z
            and abs(angular_z * TURN_RATE_CALIBRATION_FACTOR * duration) > math.pi
        ):
            raise ValueError("Rotazione oltre il limite di 180 gradi.")
        if not linear_x and not angular_z and duration > STOP_PULSE_SECONDS:
            raise ValueError("Durata del comando di stop oltre il limite configurato.")

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
            "Vettore ROS2 /cmd_vel: "
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
            raise TypeError("drive_robot ha restituito una risposta inattesa.")
        if "error" in result:
            raise RuntimeError(f"drive_robot non riuscito: {result['error']}")
        if result.get("status") != "completed":
            raise RuntimeError(
                f"Stato inatteso da drive_robot: {result.get('status')!r}."
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
        raise RuntimeError(f"Tool MCP mancanti: {sorted(missing)}")

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
        "reason": "Esplorazione completata.",
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
        raise RuntimeError(f"Tool MCP mancanti: {sorted(missing)}")

    camera_topic = await resolve_camera_topic(
        tools_by_name["list_topics"], CAMERA_TOPIC
    )
    confirmation = input(
        "Robot supervisionato e area sicura? "
        "Scrivi GO per avviare (massimo 15 minuti): "
    ).strip().upper()
    if confirmation != "GO":
        return

    await run_exploration(tools, camera_topic)


if __name__ == "__main__":
    asyncio.run(main())
