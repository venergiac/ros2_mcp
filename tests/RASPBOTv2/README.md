# RASPBOT V2 TEST

Run the following command on RASPBOTV2 via ssh

```
#!/bin/bash
docker run -it \
--privileged=true \
--net=host \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
--security-opt apparmor:unconfined \
-v /home/pi/temp:/root/temp \
-v /dev/i2c-1:/dev/i2c-1 \
-v /dev/i2c-0:/dev/i2c-0 \
--device=/dev/video0 \
--device=/dev/video1 \
--device=/dev/gpiomem \
yahboomtechnology/ros-humble:0.1.0 /bin/bash -i -c "ros2 launch yahboomcar_bringup bringup.launch.py & ros2 run usb_cam usb_cam_node_exe"
```

then start MCP server

```
docker compose up --build
```

simple navigation on RASPBOTV2

```
cd tests/RASPBOTv2
python -m pip install -r requirements.txt
python ros2-agentic-ai.py
```

`ros2-agentic-ai.py` uses four focused nodes: Explorer scans directions,
Photographer captures images, Advancer moves only the requested distance, and
the selected orchestrator routes between them. The deterministic orchestrator
is the default. Choose the Groq LLM orchestrator with `--orchestrator llm`:

```bash
export MODEL_PROVIDER=groq
export MODEL_NAME=qwen/qwen3.8-27b
export GROQ_API_KEY='<your Groq API key>'
python ros2-agentic-ai.py --orchestrator llm
```

`requirements.txt` lists the Python dependencies for the controller and notebook.
ROS 2 and the MCP server are provided separately by the robot environment and
the repository's Docker Compose setup. Explorer treats the robot's initial
heading as the relative reference and scans in groups:
`(0, -25, +50)`, then `(+50, -100)`, then `(+100)`. It stops at the first
group with safe clearance, selects that group's clearest direction, and
returns every measured direction and distance, then restores its starting
orientation. The controller realigns to the selected direction immediately
before Advancer moves. Images are saved in
`robot_photos`. ROS 2 `angular.z` is an angular velocity in rad/s, not an angle:
the controller sends `±0.25 rad/s` and computes duration from the observed
calibration that a 45° request produced about 225° of rotation. It therefore
uses a 5x calibrated rate (1.25 rad/s), so a 45° turn takes about 0.628 seconds.
The heading estimate is open-loop, so actual rotation can vary with the robot
and surface; verify it with the manual rotation test before autonomous use.
Positive ROS Z rotates counterclockwise.
During exploration it prints each incremental rotation, the resulting relative
direction, and its measured distance. Before each `/cmd_vel` call it also prints
the transmitted `linear=(x, y, z)` and `angular=(x, y, z)` vectors and duration.
The rotation calibration does not affect forward motion: `linear.x` remains
fixed at 0.05 m/s. The orchestrator computes a safe requested advance distance
from the selected clearance; Advancer converts only that distance into a
forward command. Photographer is scheduled no more than once per minute.

An interactive rotation check is available but skipped by default because it
moves the physical robot. With the robot connected and its area clear, run:

```bash
cd tests/RASPBOTv2
RUN_RASPBOT_ROTATION_TEST=1 python -m unittest -v \
  test_robot_navigation.RobotToolTests.test_manual_rotation_sequence_with_visual_confirmation
```

It requests `GO`, then commands incremental counterclockwise turns of 45°, 90°,
135°, and 180°, asking for visual confirmation after each. A negative answer
stops the sequence and sends a stop command.
