"""
BARNS Robot Arm Service

Physical robotic arm control service for the BARNS automation system.
Provides hardware interfaces, motion planning, and safety protocols.
"""

__version__ = "1.0.0"
__author__ = "BARNS Team"

from .robot_actions import ROBOT_ACTIONS

__all__ = ["ROBOT_ACTIONS"] 