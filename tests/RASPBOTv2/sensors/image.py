import asyncio
import base64
import binascii
import os
import time
from datetime import datetime, timezone
from pathlib import Path

from . import SENSOR_TIMEOUT_SECONDS, decode_tool_result

PHOTO_INTERVAL_SECONDS = 10.0
CAMERA_TOPIC = os.environ.get("ROS2_CAMERA_TOPIC")
PHOTO_DIRECTORY = Path(
    os.environ.get("ROS2_PHOTO_DIRECTORY", "./robot_photos")
).expanduser()


def _extract_image(value):
    if isinstance(value, str):
        try:
            return _extract_image(decode_tool_result(value))
        except RuntimeError:
            return None
    if isinstance(value, dict):
        mime_type = value.get("mimeType") or value.get("mime_type")
        data = value.get("base64", value.get("data"))
        if value.get("type") == "image" and data is not None and mime_type:
            if isinstance(data, bytes):
                return data, mime_type
            try:
                return base64.b64decode(data, validate=True), mime_type
            except (binascii.Error, TypeError) as exc:
                raise RuntimeError("get_image returned invalid base64 data.") from exc
        for key in ("content", "structuredContent", "text"):
            if key in value:
                image = _extract_image(value[key])
                if image is not None:
                    return image
    elif isinstance(value, (list, tuple)):
        for item in value:
            image = _extract_image(item)
            if image is not None:
                return image
    elif hasattr(value, "content"):
        return _extract_image(value.content)
    elif getattr(value, "type", None) == "image":
        return _extract_image(
            {
                "type": "image",
                "data": getattr(value, "data", None),
                "mimeType": getattr(value, "mimeType", None)
                or getattr(value, "mime_type", None),
            }
        )
    return None


def _image_extension(mime_type):
    extensions = {
        "image/jpeg": ".jpg",
        "image/png": ".png",
        "image/webp": ".webp",
    }
    try:
        return extensions[mime_type.lower()]
    except (AttributeError, KeyError) as exc:
        raise RuntimeError(f"Unsupported image MIME type: {mime_type!r}") from exc


async def resolve_camera_topic(list_topics_tool, configured_topic=None):
    result = decode_tool_result(await list_topics_tool.ainvoke({}))
    if not isinstance(result, dict):
        raise TypeError("list_topics returned an unexpected response.")
    if "error" in result:
        raise RuntimeError(f"list_topics failed: {result['error']}")

    topics = result.get("topics")
    if isinstance(topics, list):
        image_topics = [
            topic["name"]
            for topic in topics
            if isinstance(topic, dict)
            and "sensor_msgs/msg/Image" in topic.get("types", [])
        ]
    else:
        image_topics = [
            topic
            for topic in result.get("image_topics", [])
            if not topic.rstrip("/").lower().endswith(
                ("/compressed", "/compresseddepth")
            )
        ]

    if configured_topic:
        if configured_topic not in image_topics:
            raise RuntimeError(
                f"{configured_topic!r} is not an available uncompressed Image topic: "
                f"{image_topics}"
            )
        return configured_topic
    if "/image_raw" in image_topics:
        return "/image_raw"
    if len(image_topics) == 1:
        return image_topics[0]
    if not image_topics:
        raise RuntimeError("No uncompressed ROS2 Image topics are available.")
    raise RuntimeError(
        "Multiple uncompressed camera topics are available; set ROS2_CAMERA_TOPIC "
        f"to one of: {', '.join(image_topics)}"
    )


async def save_photo(image_tool, topic_name):
    result = await image_tool.ainvoke(
        {"topic_name": topic_name, "timeout_sec": SENSOR_TIMEOUT_SECONDS}
    )
    print(f"get_image result: {result}")
    image = _extract_image(result)
    if image is None:
        print("get_image returned no image content.")
        return None
    image_data, mime_type = image

    PHOTO_DIRECTORY.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    path = PHOTO_DIRECTORY / f"ros2_photo_{timestamp}{_image_extension(mime_type)}"
    path.write_bytes(image_data)
    print(f"Photo salvata: {path}")
    return path


async def capture_photos(image_tool, topic_name, stop_event, saved_paths):
    deadline = time.monotonic() + PHOTO_INTERVAL_SECONDS
    while not stop_event.is_set():
        try:
            await asyncio.wait_for(
                stop_event.wait(), timeout=max(0.0, deadline - time.monotonic())
            )
            break
        except asyncio.TimeoutError:
            saved_paths.append(await save_photo(image_tool, topic_name))
            deadline += PHOTO_INTERVAL_SECONDS
            if deadline <= time.monotonic():
                deadline = time.monotonic() + PHOTO_INTERVAL_SECONDS
