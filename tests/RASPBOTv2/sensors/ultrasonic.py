import json
import math

from . import SENSOR_TIMEOUT_SECONDS, decode_tool_result

TOPIC_NAME = "/ultrasonic"


def _find_distance(value):
    if isinstance(value, str):
        try:
            return _find_distance(json.loads(value))
        except json.JSONDecodeError:
            return None
    if isinstance(value, dict):
        if "error" in value:
            raise RuntimeError(f"topic_echo failed: {value['error']}")
        if "data" in value:
            data = value["data"]
            if isinstance(data, (int, float)) and not isinstance(data, bool):
                return float(data)
            distance = _find_distance(data)
            if distance is not None:
                return distance
        for key in ("messages", "content", "structuredContent", "text"):
            if key in value:
                distance = _find_distance(value[key])
                if distance is not None:
                    return distance
    elif isinstance(value, (list, tuple)):
        for item in value:
            distance = _find_distance(item)
            if distance is not None:
                return distance
    return None


async def read_distance(sensor):
    result = await sensor.ainvoke(
        {
            "topic_name": TOPIC_NAME,
            "message_count": 1,
            "include_arrays": False,
            "timeout_sec": SENSOR_TIMEOUT_SECONDS,
        }
    )
    print(result)
    distance = _find_distance(decode_tool_result(result))
    if distance is None or not math.isfinite(distance) or distance < 0:
        raise RuntimeError(f"No valid distance was received on {TOPIC_NAME}.")
    return distance
