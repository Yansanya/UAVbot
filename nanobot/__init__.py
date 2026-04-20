"""
UAVbot CLI (Python package name remains ``nanobot`` for imports and PyPI ``nanobot-ai``).
"""

__version__ = "0.1.5"
__logo__ = "🐈"
CLI_NAME = "uavbot"

from nanobot.nanobot import Nanobot, RunResult

__all__ = ["Nanobot", "RunResult", "CLI_NAME"]
