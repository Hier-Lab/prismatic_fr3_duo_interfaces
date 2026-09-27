"""
Public import facade for the robot motion Python API.

Applications should import this module so the API has a stable, descriptive
name while its implementation remains in :mod:`robot_api`.
"""

from robot_api import (
    RobotAPI,
    RobotAPIError,
    MoveLResult,
    RobotStateResult,
    ServiceCallTimeoutError,
    ServiceResult,
    ServiceUnavailableError,
)

__all__ = [
    "RobotAPI",
    "RobotAPIError",
    "MoveLResult",
    "RobotStateResult",
    "ServiceCallTimeoutError",
    "ServiceResult",
    "ServiceUnavailableError",
]
