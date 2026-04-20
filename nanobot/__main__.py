"""
Entry point for running the CLI as a module: ``python -m nanobot`` (same as ``uavbot`` / ``nanobot``).
"""

from nanobot.cli.commands import app

if __name__ == "__main__":
    app()
