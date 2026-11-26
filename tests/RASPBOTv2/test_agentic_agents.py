import time
import unittest

from robot_agents.advancer import create_advancer
from robot_agents.explorer import align_to_selected_direction, create_explorer
from robot_agents.orchestrator import create_orchestrator
from robot_agents.photographer import create_photographer


class SequenceTool:
    def __init__(self, values):
        self.values = iter(values)
        self.calls = 0

    async def ainvoke(self, _):
        self.calls += 1
        return next(self.values)


class AgentTests(unittest.IsolatedAsyncioTestCase):
    async def test_explorer_stops_after_first_safe_group_and_keeps_all_readings(self):
        sensor = SequenceTool([20.0, 20.0, 20.0, 35.0, 30.0, 99.0])
        rotations = []

        async def drive_robot(**command):
            rotations.append(command)

        explorer = create_explorer(
            {"read_proximity": sensor},
            drive_robot,
            turn_speed_rad_s=0.25,
            rotation_duration_seconds=lambda degrees: abs(degrees),
            minimum_clearance_cm=28.0,
        )
        result = await explorer(
            {
                "heading_degrees": 0,
                "scans": 0,
                "distance": None,
                "agent_results": {},
            }
        )

        self.assertEqual(sensor.calls, 5)
        self.assertEqual(len(result["readings"]), 5)
        self.assertEqual(result["selected_direction"], "Z=+75°")
        self.assertEqual(result["selected_clearance_cm"], 35.0)
        self.assertEqual(result["heading_degrees"], 0)
        self.assertEqual(rotations[-1], {"angular_z": 0.25, "duration": 25.0})

    async def test_explorer_restores_starting_heading_when_no_direction_is_safe(self):
        sensor = SequenceTool([20.0] * 6)
        rotations = []

        async def drive_robot(**command):
            rotations.append(command)

        explorer = create_explorer(
            {"read_proximity": sensor},
            drive_robot,
            turn_speed_rad_s=0.25,
            rotation_duration_seconds=lambda degrees: abs(degrees),
            minimum_clearance_cm=28.0,
        )
        result = await explorer(
            {
                "heading_degrees": 90,
                "scans": 0,
                "distance": None,
                "agent_results": {},
            }
        )

        self.assertEqual(sensor.calls, 6)
        self.assertIsNone(result["selected_direction"])
        self.assertEqual(result["heading_degrees"], 90)
        self.assertEqual(rotations[-1], {"angular_z": -0.25, "duration": 75.0})

    async def test_robot_aligns_with_selected_direction_before_advancing(self):
        rotations = []

        async def drive_robot(**command):
            rotations.append(command)

        heading = await align_to_selected_direction(
            90,
            75,
            drive_robot,
            turn_speed_rad_s=0.25,
            rotation_duration_seconds=lambda degrees: abs(degrees),
        )

        self.assertEqual(heading, 165)
        self.assertEqual(rotations, [{"angular_z": 0.25, "duration": 75.0}])

    async def test_advancer_moves_only_the_requested_distance(self):
        commands = []

        async def drive_robot(**command):
            commands.append(command)

        advancer = create_advancer(
            drive_robot,
            forward_speed_mps=0.05,
            maximum_advance_distance_cm=10.0,
        )
        result = await advancer(
            {"advance_distance_cm": 4.0, "agent_results": {}}
        )

        self.assertEqual(commands, [{"linear_x": 0.05, "duration": 0.8}])
        self.assertEqual(result["last_advance_distance_cm"], 4.0)

    async def test_photographer_only_captures_and_reports_photo(self):
        camera = SequenceTool(["robot_photos/test.png"])
        photographer = create_photographer({"take_photo": camera})

        result = await photographer(
            {"photos": [], "agent_results": {}}
        )

        self.assertEqual(camera.calls, 1)
        self.assertEqual(result["photos"], ["robot_photos/test.png"])
        self.assertEqual(result["agent_results"]["photographer"]["photo_path"],
                         "robot_photos/test.png")

    async def test_orchestrator_requests_safe_advance_from_clearance(self):
        orchestrator = create_orchestrator(
            max_cycles=3,
            max_duration_seconds=1000.0,
            min_photo_interval_seconds=60.0,
            minimum_clearance_cm=28.0,
            obstacle_clearance_cm=25.0,
            obstacle_safety_margin_cm=2.0,
            advance_fraction=0.5,
            maximum_advance_distance_cm=10.0,
            minimum_advance_duration_seconds=0.1,
            forward_speed_mps=0.05,
        )
        result = await orchestrator(
            {
                "cycles": 0,
                "started_at": time.monotonic(),
                "last_agent": "explorer",
                "selected_direction": "Z=+25°",
                "selected_clearance_cm": 50.0,
                "last_photo_at": time.monotonic(),
            }
        )

        self.assertEqual(result["next_action"], "advancer")
        self.assertEqual(result["advance_distance_cm"], 10.0)


if __name__ == "__main__":
    unittest.main()
