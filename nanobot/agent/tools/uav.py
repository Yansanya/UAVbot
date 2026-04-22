"""UAV tools: fly_to, get_position, get_drone_state, capture_image, analyze_scene, generate_report.

These tools communicate with the ROS-side UAV Bridge Server via HTTP.
The bridge URL defaults to http://localhost:8765 and can be overridden
via the UAV_BRIDGE_URL environment variable.
"""

from __future__ import annotations

import base64
import json
import os
import time
from pathlib import Path
from typing import Any

import httpx
from loguru import logger

from nanobot.agent.tools.base import Tool, tool_parameters
from nanobot.agent.tools.schema import (
    BooleanSchema,
    NumberSchema,
    StringSchema,
    tool_parameters_schema,
)

DEFAULT_BRIDGE_URL = "http://localhost:8765"
_CAPTURE_DIR = "uav_captures"


def _bridge_url() -> str:
    return os.environ.get("UAV_BRIDGE_URL", DEFAULT_BRIDGE_URL)


async def _get(path: str, timeout: float = 5.0) -> dict[str, Any]:
    url = f"{_bridge_url()}{path}"
    async with httpx.AsyncClient(timeout=timeout) as client:
        r = await client.get(url)
        r.raise_for_status()
        return r.json()


async def _post(path: str, body: dict[str, Any], timeout: float = 10.0) -> dict[str, Any]:
    url = f"{_bridge_url()}{path}"
    async with httpx.AsyncClient(timeout=timeout) as client:
        r = await client.post(url, json=body)
        r.raise_for_status()
        return r.json()


# ---------------------------------------------------------------------------
# 1) fly_to
# ---------------------------------------------------------------------------

@tool_parameters(
    tool_parameters_schema(
        x=NumberSchema(description="Target X coordinate (ENU east, meters)"),
        y=NumberSchema(description="Target Y coordinate (ENU north, meters)"),
        z=NumberSchema(description="Target Z coordinate (ENU up, meters). Omit to keep current altitude.", nullable=True),
        wait=BooleanSchema(description="If true, poll until the drone arrives or timeout (default true)"),
        required=["x", "y"],
    )
)
class FlyToTool(Tool):
    """Send a navigation goal to EGO-Planner and optionally wait for arrival."""

    name = "fly_to"
    description = (
        "Fly the drone to target coordinates (x, y, z) in ENU frame. "
        "x=east, y=north, z=up (meters). If z is omitted the current altitude is kept. "
        "Set wait=false to return immediately without waiting for arrival."
    )

    def __init__(self, bridge_url: str | None = None):
        if bridge_url:
            os.environ.setdefault("UAV_BRIDGE_URL", bridge_url)

    async def execute(
        self,
        x: float,
        y: float,
        z: float | None = None,
        wait: bool = True,
        **kwargs: Any,
    ) -> str:
        body: dict[str, Any] = {"x": x, "y": y}
        if z is not None:
            body["z"] = z

        try:
            resp = await _post("/fly_to", body)
        except Exception as e:
            return f"Error sending goal: {e}"

        if not wait:
            goal = resp.get("goal", body)
            return f"Goal sent: ({goal['x']}, {goal['y']}, {goal.get('z', '?')}). Not waiting."

        timeout = 120
        poll_interval = 2.0
        elapsed = 0.0
        while elapsed < timeout:
            await _async_sleep(poll_interval)
            elapsed += poll_interval
            try:
                state = await _get("/state")
                if state.get("arrived"):
                    pos = await _get("/position")
                    return (
                        f"Arrived at ({pos['x']}, {pos['y']}, {pos['z']}) "
                        f"after {elapsed:.0f}s."
                    )
                status = state.get("status", "UNKNOWN")
                if status in ("IDLE", "NO_ODOM"):
                    return f"Drone status is {status} — goal may have been lost."
            except Exception as e:
                logger.debug("poll error: {}", e)

        return f"Timeout after {timeout}s. Drone may still be en route."


# ---------------------------------------------------------------------------
# 2) get_position
# ---------------------------------------------------------------------------

@tool_parameters(tool_parameters_schema())
class GetPositionTool(Tool):
    """Get the drone's current position."""

    name = "get_position"
    description = "Return the drone's current position (x, y, z, yaw) in the ENU world frame."

    def __init__(self, bridge_url: str | None = None):
        if bridge_url:
            os.environ.setdefault("UAV_BRIDGE_URL", bridge_url)

    @property
    def read_only(self) -> bool:
        return True

    async def execute(self, **kwargs: Any) -> str:
        try:
            data = await _get("/position")
            if not data.get("has_odom"):
                return "No odometry data available yet."
            return json.dumps(data, ensure_ascii=False)
        except Exception as e:
            return f"Error: {e}"


# ---------------------------------------------------------------------------
# 3) get_drone_state
# ---------------------------------------------------------------------------

@tool_parameters(tool_parameters_schema())
class GetDroneStateTool(Tool):
    """Get the drone's flight state."""

    name = "get_drone_state"
    description = (
        "Return the drone's current state: HTTP bridge status (IDLE/FLYING/ARRIVED/NO_ODOM), "
        "goal_active, arrived, distance_m when applicable, and ego_planner when available "
        "(EGO-Planner FSM: fsm, have_odom, have_target, trigger, wait_for_goal from /ego_planner/fsm_status)."
    )

    def __init__(self, bridge_url: str | None = None):
        if bridge_url:
            os.environ.setdefault("UAV_BRIDGE_URL", bridge_url)

    @property
    def read_only(self) -> bool:
        return True

    async def execute(self, **kwargs: Any) -> str:
        try:
            data = await _get("/state")
            return json.dumps(data, ensure_ascii=False)
        except Exception as e:
            return f"Error: {e}"


# ---------------------------------------------------------------------------
# 4) capture_image
# ---------------------------------------------------------------------------

@tool_parameters(
    tool_parameters_schema(
        filename=StringSchema("Optional filename (saved under uav_captures/)", nullable=True),
    )
)
class CaptureImageTool(Tool):
    """Capture a camera image from the drone."""

    name = "capture_image"
    description = (
        "Capture an image from the drone's front camera via AirSim. "
        "Returns the saved file path. The image can then be analyzed with analyze_scene."
    )

    def __init__(self, workspace: Path | None = None, bridge_url: str | None = None):
        self._workspace = workspace or Path(os.environ.get("NANOBOT_WORKSPACE", "~/.nanobot/workspace")).expanduser()
        if bridge_url:
            os.environ.setdefault("UAV_BRIDGE_URL", bridge_url)

    async def execute(self, filename: str | None = None, **kwargs: Any) -> str:
        try:
            data = await _get("/capture", timeout=10.0)
        except Exception as e:
            return f"Error capturing image: {e}"

        if not data.get("ok"):
            return f"Capture failed: {data.get('error', 'unknown')}"

        capture_dir = self._workspace / _CAPTURE_DIR
        capture_dir.mkdir(parents=True, exist_ok=True)

        if not filename:
            filename = f"capture_{int(time.time())}.png"
        filepath = capture_dir / filename

        img_bytes = base64.b64decode(data["image_base64"])
        filepath.write_bytes(img_bytes)

        return f"Image saved: {filepath} ({data['width']}x{data['height']})"


# ---------------------------------------------------------------------------
# 5) analyze_scene
# ---------------------------------------------------------------------------

@tool_parameters(
    tool_parameters_schema(
        prompt=StringSchema("What to look for or analyze in the image", nullable=True),
    )
)
class AnalyzeSceneTool(Tool):
    """Capture an image and analyze it with the LLM's vision capability."""

    name = "analyze_scene"
    description = (
        "Capture a drone camera image and analyze the scene using vision AI. "
        "Optionally provide a prompt to focus the analysis (e.g. 'Are there any cracks?'). "
        "Returns a text description of what the camera sees."
    )

    def __init__(self, workspace: Path | None = None, bridge_url: str | None = None):
        self._workspace = workspace or Path(os.environ.get("NANOBOT_WORKSPACE", "~/.nanobot/workspace")).expanduser()
        if bridge_url:
            os.environ.setdefault("UAV_BRIDGE_URL", bridge_url)

    async def execute(self, prompt: str | None = None, **kwargs: Any) -> Any:
        try:
            data = await _get("/capture", timeout=10.0)
        except Exception as e:
            return f"Error capturing image: {e}"

        if not data.get("ok"):
            return f"Capture failed: {data.get('error', 'unknown')}"

        capture_dir = self._workspace / _CAPTURE_DIR
        capture_dir.mkdir(parents=True, exist_ok=True)
        filename = f"analyze_{int(time.time())}.png"
        filepath = capture_dir / filename

        img_bytes = base64.b64decode(data["image_base64"])
        filepath.write_bytes(img_bytes)

        b64_url = f"data:image/png;base64,{data['image_base64']}"
        user_prompt = prompt or "Describe what you see in this drone camera image in detail. Note any objects, terrain features, or anomalies."

        return [
            {"type": "text", "text": f"[Image saved: {filepath}]\n\nAnalyzing scene..."},
            {
                "type": "image_url",
                "image_url": {"url": b64_url},
            },
            {"type": "text", "text": f"Analysis request: {user_prompt}"},
        ]


# ---------------------------------------------------------------------------
# 6) generate_report
# ---------------------------------------------------------------------------

@tool_parameters(
    tool_parameters_schema(
        title=StringSchema("Report title"),
        findings=StringSchema("Summary of findings from the inspection"),
        required=["title", "findings"],
    )
)
class GenerateReportTool(Tool):
    """Generate a Markdown inspection report."""

    name = "generate_report"
    description = (
        "Generate a Markdown inspection report. Provide a title and a summary of findings. "
        "The report includes all captured images from this session."
    )

    def __init__(self, workspace: Path | None = None, bridge_url: str | None = None):
        self._workspace = workspace or Path(os.environ.get("NANOBOT_WORKSPACE", "~/.nanobot/workspace")).expanduser()
        if bridge_url:
            os.environ.setdefault("UAV_BRIDGE_URL", bridge_url)

    async def execute(self, title: str, findings: str, **kwargs: Any) -> str:
        capture_dir = self._workspace / _CAPTURE_DIR
        images = sorted(capture_dir.glob("*.png")) if capture_dir.exists() else []

        lines = [
            f"# {title}",
            "",
            f"**Date**: {time.strftime('%Y-%m-%d %H:%M:%S')}",
            "",
            "## Findings",
            "",
            findings,
            "",
        ]

        if images:
            lines.append("## Captured Images")
            lines.append("")
            for img in images:
                lines.append(f"### {img.stem}")
                lines.append(f"![{img.stem}]({img})")
                lines.append("")

        lines.append("---")
        lines.append("*Report generated by UAVbot*")

        report_content = "\n".join(lines)
        report_dir = self._workspace / "reports"
        report_dir.mkdir(parents=True, exist_ok=True)
        report_path = report_dir / f"report_{int(time.time())}.md"
        report_path.write_text(report_content, encoding="utf-8")

        return f"Report saved: {report_path}\n\n{report_content}"


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

async def _async_sleep(seconds: float) -> None:
    import asyncio
    await asyncio.sleep(seconds)
