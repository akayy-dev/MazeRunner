from enum import Enum


class Direction(Enum):
	"""Direction of the robot (relative to the orientation)"""
	FORWARD = 1,
	LEFT = 2,
	RIGHT = 3,
	BACKWARD = 4

class Orientation(Enum):
	"""Orientation of the Robot"""
	NORTH = 1,
	EAST  = 2,
	WEST  = 3,
	SOUTH = 4

class Side(Enum):
	"""The 'side' that the a wall is on (from top down position)"""
	TOP = 1,
	DOWN = 2,
	LEFT = 3,
	RIGHT = 4,
