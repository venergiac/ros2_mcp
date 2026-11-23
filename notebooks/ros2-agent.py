import asyncio
import math
import os
import time

from langchain_mcp_adapters.client import MultiServerMCPClient

from sensors import decode_tool_result
from sensors.image import (
    CAMERA_TOPIC,
    PHOTO_DIRECTORY,
    PHOTO_INTERVAL_SECONDS,
    capture_photos,
    resolve_camera_topic,
)
from sensors.ultrasonic import read_distance

MIN_CLEARANCE_CM = 25.0
BLIND_ZONE_CM = 3.0
MAX_DURATION_SECONDS = 15 * 60.0
MAX_CYCLES = 1800

FORWARD_SPEED_MPS = 0.10
FORWARD_PULSE_SECONDS = 0.5
MAX_STRAIGHT_PULSES = 8
TURN_SPEED_PERCENT = 20.0
TURN_PULSE_SECONDS = 0.25
STOP_PULSE_SECONDS = 0.2
SCAN_RAY_COUNT = 8
SCAN_STEP_DEGREES = 45


async def explore(tools):
    tools_by_name = {tool.name: tool for tool in tools}
    required = {"topic_echo", "drive_robot"}
    missing = required - tools_by_name.keys()
    if missing:
        raise RuntimeError(f"Missing required MCP tools: {sorted(missing)}")
    sensor = tools_by_name["topic_echo"]
    driver = tools_by_name["drive_robot"]

    async def drive(linear_x=0.0, angular_z=0.0, duration=STOP_PULSE_SECONDS):
        if not all(math.isfinite(value) for value in (linear_x, angular_z, duration)):
            raise ValueError("Motion values must be finite.")
        if abs(linear_x) > FORWARD_SPEED_MPS:
            raise ValueError("Forward speed exceeds its configured limit.")
        if abs(angular_z) > TURN_SPEED_PERCENT:
            raise ValueError("Turn command exceeds its 20-percent limit.")
        if linear_x and angular_z:
            raise ValueError("Combined translation and rotation is not supported.")
        if linear_x and duration > FORWARD_PULSE_SECONDS:
            raise ValueError("Forward pulse exceeds its configured maximum.")
        if angular_z and duration > TURN_PULSE_SECONDS:
            raise ValueError("Rotation pulse exceeds its configured maximum.")
        if not linear_x and not angular_z and duration > STOP_PULSE_SECONDS:
            raise ValueError("Stop command exceeds its configured maximum.")

        result = await driver.ainvoke(
            {
                "topic_name": "/cmd_vel",
                "linear_x": linear_x,
                "linear_y": 0.0,
                "linear_z": 0.0,
                "angular_x": 0.0,
                "angular_y": 0.0,
                "angular_z": angular_z,
                "duration_sec": duration,
            }
        )
        result = decode_tool_result(result)
        if not isinstance(result, dict):
            raise TypeError("drive_robot returned an unexpected response.")
        if "error" in result:
            raise RuntimeError(f"drive_robot failed: {result['error']}")
        if result.get("status") != "completed":
            raise RuntimeError(f"drive_robot returned status {result.get('status')!r}.")

    async def rotate_45(direction):
        if direction not in (-1, 1):
            raise ValueError("Direction must be -1 (right) or 1 (left).")
        await drive(angular_z=direction * TURN_SPEED_PERCENT, duration=TURN_PULSE_SECONDS)

    async def scan_sides(front_distance):
        readings = [front_distance]
        for _ in range(1, SCAN_RAY_COUNT):
            await rotate_45(1)
            readings.append(await read_distance(sensor))
        await rotate_45(1)

        clear_rays = [
            (index, distance)
            for index, distance in enumerate(readings)
            if distance > MIN_CLEARANCE_CM
        ]
        if not clear_rays:
            for _ in range(SCAN_RAY_COUNT // 2):
                await rotate_45(1)
            return None

        best_index, _ = max(clear_rays, key=lambda ray: ray[1])
        steps = min(best_index, SCAN_RAY_COUNT - best_index)
        direction = 1 if best_index <= SCAN_RAY_COUNT // 2 else -1
        for _ in range(steps):
            await rotate_45(direction)
        return best_index * SCAN_STEP_DEGREES

    started = time.monotonic()
    cycles = forward_steps = scans = turns = 0
    reversed_from_block = False
    status = "TIME_LIMIT"
    reason = "Exploration time limit reached."
    try:
        while cycles < MAX_CYCLES and time.monotonic() - started < MAX_DURATION_SECONDS:
            distance = await read_distance(sensor)
            cycles += 1
            if distance <= MIN_CLEARANCE_CM:
                if reversed_from_block:
                    status, reason = "BLOCKED", "Still blocked after turning around."
                    break
                if distance <= BLIND_ZONE_CM:
                    print("Front reading is in the blind zone; scanning all directions.")
                direction = await scan_sides(distance)
                scans += 1
                print(
                    f"Scan {scans}: front={distance:.1f} cm, "
                    f"selected_angle={direction}"
                )
                reversed_from_block = direction is None
                forward_steps = 0
                continue

            reversed_from_block = False
            if forward_steps >= MAX_STRAIGHT_PULSES:
                await rotate_45(1 if turns % 2 == 0 else -1)
                turns += 1
                forward_steps = 0
            else:
                await drive(FORWARD_SPEED_MPS, duration=FORWARD_PULSE_SECONDS)
                forward_steps += 1
        else:
            if cycles >= MAX_CYCLES:
                status, reason = "CYCLE_LIMIT", "Exploration cycle limit reached."
    finally:
        await drive(duration=STOP_PULSE_SECONDS)

    return {
        "status": status,
        "reason": reason,
        "cycles": cycles,
        "obstacle_scans": scans,
    }


async def run_exploration_with_photos(tools, camera_topic):
    stop_event = asyncio.Event()
    saved_paths = []
    tools_by_name = {tool.name: tool for tool in tools}
    image_tool = tools_by_name["get_image"]
    photo_task = asyncio.create_task(
        capture_photos(image_tool, camera_topic, stop_event, saved_paths)
    )
    navigation_task = asyncio.create_task(explore(tools))
    try:
        done, _ = await asyncio.wait(
            (navigation_task, photo_task), return_when=asyncio.FIRST_COMPLETED
        )
        if photo_task in done:
            await photo_task
        return await navigation_task, saved_paths
    except BaseException:
        navigation_task.cancel()
        await asyncio.gather(navigation_task, return_exceptions=True)
        raise
    finally:
        stop_event.set()
        if not photo_task.done():
            await photo_task


async def main():
    mcp_url = os.environ.get("ROS2_MCP_URL", "http://localhost:8000/sse")
    client = MultiServerMCPClient(
        {"ros2_mcp": {"transport": "sse", "url": mcp_url}}
    )
    tools = await client.get_tools()
    tools_by_name = {tool.name: tool for tool in tools}
    required = {"list_topics", "get_image", "topic_echo", "drive_robot"}
    missing = required - tools_by_name.keys()
    if missing:
        raise RuntimeError(f"Missing required MCP tools: {sorted(missing)}")

    camera_topic = await resolve_camera_topic(
        tools_by_name["list_topics"], CAMERA_TOPIC
    )
    print(
        f"Camera: {camera_topic}; photos every {PHOTO_INTERVAL_SECONDS:.0f} seconds "
        f"under {PHOTO_DIRECTORY}."
    )
    confirmation = input(
        "Robot supervised, area authorized, occupants informed? "
        "Type GO to explore for at most 15 minutes: "
    ).strip().upper()
    if confirmation != "GO":
        print("Exploration cancelled; no movement command sent.")
        return

    result, photos = await run_exploration_with_photos(tools, camera_topic)
    print({**result, "photos_saved": len(photos), "photo_directory": str(PHOTO_DIRECTORY)})


if __name__ == "__main__":
    asyncio.run(main())
