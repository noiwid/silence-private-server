"""Shared fixtures and path setup for the test suite."""
import os
import sys

# Make the project root importable so tests can do
# `from helpers.messageParser import MessageParser`.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))


def change_to_repo_root():
    """Change CWD to repo root — MessageParser opens
    'scooter_status_definition.json' with a relative path."""
    os.chdir(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))


# Always run from repo root so the parser can open its JSON configs.
change_to_repo_root()
