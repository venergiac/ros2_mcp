import json

SENSOR_TIMEOUT_SECONDS = 1.5


def decode_tool_result(value):
    if isinstance(value, str):
        try:
            return json.loads(value)
        except json.JSONDecodeError as exc:
            raise RuntimeError("MCP tool returned invalid JSON.") from exc
    if isinstance(value, dict):
        if "status" in value or "error" in value:
            return value
        for key in ("structuredContent", "content", "text"):
            if key in value:
                return decode_tool_result(value[key])
    if hasattr(value, "text"):
        return decode_tool_result(value.text)
    if hasattr(value, "content"):
        return decode_tool_result(value.content)
    if isinstance(value, list) and len(value) == 1:
        return decode_tool_result(value[0])
    return value
