from enum import Enum


class Direction(Enum):
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
