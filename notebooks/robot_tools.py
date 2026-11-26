from contextlib import redirect_stdout
from io import StringIO

from langchain_core.tools import BaseTool, tool

from sensors.image import save_photo
from sensors.ultrasonic import read_distance


def create_robot_tools(
    sensor_tool: BaseTool,
    image_tool: BaseTool,
    camera_topic: str,
) -> dict[str, BaseTool]:
    @tool(
        "read_proximity",
        description="Read and validate one measurement from the robot proximity sensor.",
    )
    async def read_proximity() -> float:
        with redirect_stdout(StringIO()):
            return await read_distance(sensor_tool)

    @tool(
        "take_photo",
        description="Capture a camera image and save it under the robot_photos directory.",
    )
    async def take_photo() -> str:
        with redirect_stdout(StringIO()):
            photo_path = await save_photo(image_tool, camera_topic)
        if photo_path is None:
            raise RuntimeError("The camera tool returned no image.")
        return str(photo_path)

    return {
        "read_proximity": read_proximity,
        "take_photo": take_photo,
    }
