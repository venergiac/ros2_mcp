import importlib.util
import io
import math
import os
import sys
import time
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import AsyncMock, patch

NOTEBOOK_DIRECTORY = Path(__file__).parent
sys.path.insert(0, str(NOTEBOOK_DIRECTORY))

from robot_tools import create_robot_tools

AGENT_PATH = NOTEBOOK_DIRECTORY / "ros2-agent-ai.py"
AGENT_SPEC = importlib.util.spec_from_file_location("ros2_agent_ai", AGENT_PATH)
agent = importlib.util.module_from_spec(AGENT_SPEC)
AGENT_SPEC.loader.exec_module(agent)


async def invoke_graph(graph, state):
    output = io.StringIO()
    with redirect_stdout(output):
        result = await graph.ainvoke(state)
    return result, output.getvalue()


class FakeTool:
    def __init__(self, name, result):
        self.name = name
        self.result = result
        self.calls = []

    async def ainvoke(self, payload):
        self.calls.append(payload)
        return self.result


class SequenceTool(FakeTool):
    def __init__(self, name, results, default_result=None):
        super().__init__(name, None)
        self.results = iter(results)
        self.default_result = default_result

    async def ainvoke(self, payload):
        self.calls.append(payload)
        try:
            return next(self.results)
        except StopIteration:
            if self.default_result is None:
                raise
            return self.default_result


def initial_state():
    return {
        "next_agent": "explorer",
        "status": "RUNNING",
        "reason": "",
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


class DirectionSelectionTests(unittest.TestCase):
    def test_selects_clear_direction_with_most_space(self):
        readings = {
            "Z=-25°": 100.0,
            "Z=+25°": 150.0,
        }
        self.assertEqual(agent.select_direction(readings), 25)

    def test_rejects_directions_without_a_safe_forward_step(self):
        readings = {"Z=+25°": 27.9, "Z=+0°": 28.0}
        self.assertIsNone(agent.select_direction(readings))

    def test_rotation_duration_uses_requested_degrees_and_ros_angular_velocity(self):
        for angle in (45, 90, 135, 180):
            with self.subTest(angle=angle):
                duration = agent.rotation_duration_seconds(angle)
                expected_duration = (
                    math.radians(angle) / agent.CALIBRATED_TURN_RATE_RAD_S
                )
                self.assertAlmostEqual(duration, expected_duration)
                self.assertAlmostEqual(
                    agent.CALIBRATED_TURN_RATE_RAD_S * duration,
                    math.radians(angle),
                )

    def test_forward_step_count_scales_with_clearance_and_is_capped(self):
        self.assertEqual(
            agent.forward_steps_for_clearance(agent.MIN_ADVANCE_DISTANCE_CM),
            0,
        )
        self.assertEqual(
            agent.forward_steps_for_clearance(agent.MIN_ADVANCE_DISTANCE_CM + 5),
            1,
        )
        self.assertEqual(
            agent.forward_steps_for_clearance(agent.MIN_ADVANCE_DISTANCE_CM + 15),
            3,
        )
        self.assertEqual(
            agent.forward_steps_for_clearance(200),
            agent.MAX_FORWARD_STEPS_PER_ADVANCE,
        )

class RobotToolTests(unittest.IsolatedAsyncioTestCase):
    async def test_tools_wrap_existing_sensor_and_photo_packages(self):
        async def noisy_read_distance(_sensor):
            sys.stdout.write("raw sensor response\n")
            return 42.0

        async def noisy_save_photo(_image, _topic):
            sys.stdout.write("saved photo path\n")
            return "robot_photos/test.jpg"

        with patch(
            "robot_tools.read_distance", new=AsyncMock(side_effect=noisy_read_distance)
        ), patch(
            "robot_tools.save_photo", new=AsyncMock(side_effect=noisy_save_photo)
        ):
            tools = create_robot_tools(object(), object(), "/camera")
            output = io.StringIO()
            with redirect_stdout(output):
                self.assertEqual(await tools["read_proximity"].ainvoke({}), 42.0)
                self.assertEqual(
                    await tools["take_photo"].ainvoke({}),
                    "robot_photos/test.jpg",
                )
            self.assertEqual(output.getvalue(), "")

    async def test_driver_only_sends_bounded_forward_and_stop_commands(self):
        driver_tool = FakeTool("drive_robot", {"status": "completed"})
        drive_robot = agent.create_driver(driver_tool)

        output = io.StringIO()
        with redirect_stdout(output):
            await drive_robot(linear_x=agent.FORWARD_SPEED_MPS, duration=0.1)
            await drive_robot()

        with self.assertRaises(ValueError):
            await drive_robot(linear_x=-agent.FORWARD_SPEED_MPS, duration=0.1)
        with self.assertRaises(ValueError):
            await drive_robot(
                linear_x=agent.FORWARD_SPEED_MPS,
                angular_z=agent.TURN_ANGULAR_SPEED_RAD_S,
                duration=0.1,
            )
        with self.assertRaises(ValueError):
            await drive_robot(
                angular_z=agent.MAX_ANGULAR_SPEED_RAD_S + 0.01,
                duration=0.1,
            )

        self.assertEqual(len(driver_tool.calls), 2)
        self.assertEqual(driver_tool.calls[0]["linear_x"], agent.FORWARD_SPEED_MPS)
        self.assertEqual(driver_tool.calls[1]["duration_sec"], agent.STOP_PULSE_SECONDS)
        self.assertEqual(
            output.getvalue().splitlines(),
            [
                "Vettore ROS2 /cmd_vel: "
                "linear=(0.05, 0.0, 0.0), angular=(0.0, 0.0, 0.0), "
                "duration=0.1s",
                "Vettore ROS2 /cmd_vel: "
                "linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0), "
                "duration=0.2s",
            ],
        )

    @unittest.skipUnless(
        os.environ.get("RUN_RASPBOT_ROTATION_TEST") == "1",
        "Manual hardware test; set RUN_RASPBOT_ROTATION_TEST=1 to enable.",
    )
    async def test_manual_rotation_sequence_with_visual_confirmation(self):
        client = agent.MultiServerMCPClient(
            {
                "ros2_mcp": {
                    "transport": "sse",
                    "url": os.environ.get(
                        "ROS2_MCP_URL",
                        "http://localhost:8000/sse",
                    ),
                }
            }
        )
        tools = await client.get_tools()
        driver_tool = next(
            (tool for tool in tools if tool.name == "drive_robot"),
            None,
        )
        if driver_tool is None:
            self.fail("ROS2 MCP non espone il tool drive_robot.")

        if input(
            "Verifica che l'area sia libera; scrivi GO per avviare: "
        ).strip().upper() != "GO":
            self.skipTest("Test manuale annullato.")

        drive_robot = agent.create_driver(driver_tool)
        try:
            for angle in (45, 90, 135, 180):
                await drive_robot(
                    angular_z=agent.TURN_ANGULAR_SPEED_RAD_S,
                    duration=agent.rotation_duration_seconds(angle),
                )
                confirmation = input(
                    f"Rotazione incrementale +{angle}°: corretta? [s/N] "
                ).strip().lower()
                if confirmation not in {"s", "si", "sì", "y", "yes"}:
                    self.fail(
                        f"Conferma visiva negativa o mancante dopo +{angle}°."
                    )
        finally:
            await drive_robot()


class NavigationGraphTests(unittest.IsolatedAsyncioTestCase):
    async def test_blocked_scan_never_sends_forward_motion(self):
        sensor = FakeTool("read_proximity", 20.0)
        photo = FakeTool("take_photo", "robot_photos/blocked.jpg")
        commands = []

        async def drive_robot(**command):
            commands.append(command)

        state, output = await invoke_graph(agent.create_agent_graph(
            {"read_proximity": sensor, "take_photo": photo}, drive_robot
        ), initial_state())

        self.assertEqual(state["status"], "BLOCKED")
        self.assertEqual(state["scans"], 1)
        self.assertEqual(state["moves"], 0)
        self.assertEqual(len(sensor.calls), 5)
        self.assertEqual(len(state["photos"]), 0)
        self.assertEqual(photo.calls, [])
        self.assertFalse(any(command.get("linear_x", 0) > 0 for command in commands))
        self.assertEqual(
            output.splitlines(),
            [
                "Rotazione incrementale -25° (direzione Z=-25°): "
                "distanza misurata 20.0 cm",
                "Rotazione incrementale +50° (direzione Z=+25°): "
                "distanza misurata 20.0 cm",
                "Rotazione incrementale +50° (direzione Z=+75°): "
                "distanza misurata 20.0 cm",
                "Rotazione incrementale -100° (direzione Z=-25°): "
                "distanza misurata 20.0 cm",
                "Rotazione incrementale +100° (direzione Z=+75°): "
                "distanza misurata 20.0 cm",
            ],
        )

    async def test_graph_scans_advances_and_photographs_one_cycle(self):
        events = []

        class EventPhotoTool(FakeTool):
            async def ainvoke(self, payload):
                events.append("photo")
                return await super().ainvoke(payload)

        photo = EventPhotoTool("take_photo", "robot_photos/step.jpg")
        heading = 0.0
        sampled_headings = []
        forward_commands = []
        turn_commands = []

        class HeadingSensor(SequenceTool):
            def __init__(self):
                super().__init__("read_proximity", [100.0, 150.0], 100.0)

            async def ainvoke(self, payload):
                sampled_headings.append(round(heading) % 360)
                return await super().ainvoke(payload)

        sensor = HeadingSensor()

        async def drive_robot(linear_x=0.0, angular_z=0.0, duration=0.2):
            nonlocal heading
            angular_rate = math.copysign(
                agent.CALIBRATED_TURN_RATE_RAD_S,
                angular_z,
            ) if angular_z else 0.0
            heading = (heading + math.degrees(angular_rate * duration)) % 360
            if angular_z:
                turn_commands.append((angular_z, duration))
            if linear_x:
                forward_commands.append(linear_x)
                events.append("advance")

        with patch.object(agent, "MAX_CYCLES", 1):
            state, output = await invoke_graph(agent.create_agent_graph(
                {"read_proximity": sensor, "take_photo": photo}, drive_robot
            ), initial_state())

        self.assertEqual(state["status"], "CYCLE_LIMIT")
        self.assertEqual(state["selected_direction"], "Z=+25°")
        self.assertEqual(state["selected_clearance_cm"], 150.0)
        self.assertEqual(
            state["moves"],
            agent.MAX_FORWARD_STEPS_PER_ADVANCE,
        )
        self.assertEqual(state["scans"], 1)
        self.assertEqual(len(state["photos"]), 1)
        self.assertEqual(
            forward_commands,
            [agent.FORWARD_SPEED_MPS] * agent.MAX_FORWARD_STEPS_PER_ADVANCE,
        )
        self.assertEqual(
            len(sensor.calls),
            3 + agent.MAX_FORWARD_STEPS_PER_ADVANCE,
        )
        self.assertEqual(
            sampled_headings[:8],
            [335, 25, *([25] * 6)],
        )
        self.assertTrue(math.isclose(heading, 25.0, abs_tol=1e-8))
        self.assertEqual(len(turn_commands), 2)
        self.assertLess(turn_commands[0][0], 0)
        self.assertGreater(turn_commands[1][0], 0)
        self.assertAlmostEqual(
            abs(turn_commands[0][0]),
            agent.TURN_ANGULAR_SPEED_RAD_S,
        )
        self.assertAlmostEqual(
            abs(turn_commands[1][0]),
            agent.TURN_ANGULAR_SPEED_RAD_S,
        )
        self.assertAlmostEqual(
            turn_commands[0][1],
            agent.rotation_duration_seconds(25),
        )
        self.assertAlmostEqual(
            turn_commands[1][1],
            agent.rotation_duration_seconds(50),
        )
        self.assertEqual(events[0], "photo")
        self.assertLess(events.index("photo"), events.index("advance"))
        self.assertEqual(
            output.splitlines(),
            [
                "Rotazione incrementale -25° (direzione Z=-25°): "
                "distanza misurata 100.0 cm",
                "Rotazione incrementale +50° (direzione Z=+25°): "
                "distanza misurata 150.0 cm",
            ],
        )

    async def test_forward_loop_rechecks_proximity_before_each_step(self):
        sensor = SequenceTool("read_proximity", [100.0] * 4 + [20.0])
        photo = FakeTool("take_photo", "robot_photos/step.jpg")
        forward_commands = []

        async def drive_robot(linear_x=0.0, angular_z=0.0, duration=0.2):
            if linear_x:
                forward_commands.append(linear_x)

        with patch.object(agent, "MAX_CYCLES", 1):
            state, _ = await invoke_graph(agent.create_agent_graph(
                {"read_proximity": sensor, "take_photo": photo}, drive_robot
            ), initial_state())

        self.assertEqual(
            state["status"],
            "CYCLE_LIMIT",
        )
        self.assertEqual(state["moves"], 1)
        self.assertEqual(forward_commands, [agent.FORWARD_SPEED_MPS])
        self.assertEqual(len(sensor.calls), 5)

    async def test_explorer_falls_back_and_selects_clearer_direction_in_group(self):
        sensor = SequenceTool(
            "read_proximity",
            [20.0, 20.0, 80.0, 120.0],
            default_result=100.0,
        )
        photo = FakeTool("take_photo", "robot_photos/step.jpg")

        async def drive_robot(**command):
            return None

        with patch.object(agent, "MAX_CYCLES", 1):
            state, _ = await invoke_graph(agent.create_agent_graph(
                {"read_proximity": sensor, "take_photo": photo}, drive_robot
            ), initial_state())

        self.assertEqual(state["selected_direction"], "Z=-25°")
        self.assertEqual(state["selected_clearance_cm"], 120.0)
        self.assertEqual(
            list(state["readings"]),
            ["Z=-25°", "Z=+25°", "Z=+75°"],
        )

    async def test_skips_photo_until_one_minute_has_elapsed(self):
        sensor = FakeTool("read_proximity", 100.0)
        photo = FakeTool("take_photo", "robot_photos/step.jpg")
        state = initial_state()
        now = time.monotonic()
        state["started_at"] = now
        state["last_photo_at"] = now - agent.MIN_PHOTO_INTERVAL_SECONDS + 1

        async def drive_robot(**command):
            return None

        with patch.object(agent, "MAX_CYCLES", 1):
            result, _ = await invoke_graph(agent.create_agent_graph(
                {"read_proximity": sensor, "take_photo": photo}, drive_robot
            ), state)

        self.assertEqual(result["status"], "CYCLE_LIMIT")
        self.assertEqual(result["photos"], [])
        self.assertEqual(photo.calls, [])
        self.assertGreater(result["moves"], 0)


if __name__ == "__main__":
    unittest.main()
