from typing import List
import API
from queue import Queue
import sys
from dataclasses import dataclass, field
from maze import *
from time import sleep
from orientation import *

global currentX
global currentY

currentX = 0
currentY = 0

class Robot:
	orientation: Orientation
	def __init__(self):
		pass

robot = Robot()



MAZE = [[
	Cell(False, 255, None, x, y) for x in range(0, 5)
	] for y in range(0, 5)]

def log(string: str):
	sys.stderr.write("{}\n".format(string))
	sys.stderr.flush()

def moveDirection(dir: Direction):
	global currentX, currentY
	log(f"Moving {dir} from {robot.orientation}")

	# 1. Handle Rotation (Update Orientation)
	if dir == Direction.BACKWARD:
		API.turnLeft90()
		API.turnLeft90()
		if robot.orientation == Orientation.NORTH:
			robot.orientation = Orientation.SOUTH
		elif robot.orientation == Orientation.SOUTH:
			robot.orientation = Orientation.NORTH
		elif robot.orientation == Orientation.EAST:
			robot.orientation = Orientation.WEST
		elif robot.orientation == Orientation.WEST:
			robot.orientation = Orientation.EAST

	elif dir == Direction.LEFT:
		API.turnLeft()
		if robot.orientation == Orientation.NORTH:
			robot.orientation = Orientation.WEST
		elif robot.orientation == Orientation.SOUTH:
			robot.orientation = Orientation.EAST
		elif robot.orientation == Orientation.EAST:
			robot.orientation = Orientation.NORTH
		elif robot.orientation == Orientation.WEST:
			robot.orientation = Orientation.SOUTH

	elif dir == Direction.RIGHT:
		API.turnRight()
		if robot.orientation == Orientation.NORTH:
			robot.orientation = Orientation.EAST
		elif robot.orientation == Orientation.SOUTH:
			robot.orientation = Orientation.WEST
		elif robot.orientation == Orientation.EAST:
			robot.orientation = Orientation.SOUTH
		elif robot.orientation == Orientation.WEST:
			robot.orientation = Orientation.NORTH
	
	# 2. Move Forward (Physical move)
	API.moveForward()

	# 3. Update Coordinates based on the NEW orientation
	if robot.orientation == Orientation.NORTH:
		currentY += 1
	elif robot.orientation == Orientation.SOUTH:
		currentY -= 1
	elif robot.orientation == Orientation.EAST:
		currentX += 1
	elif robot.orientation == Orientation.WEST:
		currentX -= 1

def turn_to(target: Orientation):
	if robot.orientation == target:
		return

	# Calculate turns needed
	# North=0, East=1, South=2, West=3 (Assuming this mapping for logic)
	# We can just use simple if/else for the enum
	
	# Simple logic: just turn right until we match
	while robot.orientation != target:
		API.turnRight()
		if robot.orientation == Orientation.NORTH: robot.orientation = Orientation.EAST
		elif robot.orientation == Orientation.EAST: robot.orientation = Orientation.SOUTH
		elif robot.orientation == Orientation.SOUTH: robot.orientation = Orientation.WEST
		elif robot.orientation == Orientation.WEST: robot.orientation = Orientation.NORTH

def get_side_from_direction(dir: Direction) -> Side:
	"""Returns the absolute Side based on relative direction and current orientation."""
	if robot.orientation == Orientation.NORTH:
		if dir == Direction.FORWARD: return Side.TOP
		if dir == Direction.BACKWARD: return Side.DOWN
		if dir == Direction.LEFT: return Side.LEFT
		if dir == Direction.RIGHT: return Side.RIGHT
	
	elif robot.orientation == Orientation.EAST:
		if dir == Direction.FORWARD: return Side.RIGHT
		if dir == Direction.BACKWARD: return Side.LEFT
		if dir == Direction.LEFT: return Side.TOP
		if dir == Direction.RIGHT: return Side.DOWN

	elif robot.orientation == Orientation.SOUTH:
		if dir == Direction.FORWARD: return Side.DOWN
		if dir == Direction.BACKWARD: return Side.TOP
		if dir == Direction.LEFT: return Side.RIGHT
		if dir == Direction.RIGHT: return Side.LEFT

	elif robot.orientation == Orientation.WEST:
		if dir == Direction.FORWARD: return Side.LEFT
		if dir == Direction.BACKWARD: return Side.RIGHT
		if dir == Direction.RIGHT: return Side.TOP
		if dir == Direction.LEFT: return Side.DOWN



def get_neighbor_coords(dir: Direction) -> List[int]:
	"""Return X and Y coords depending on direction"""
	nx, ny = currentX, currentY

	target_orient = robot.orientation
	if dir == Direction.LEFT:
		if robot.orientation == Orientation.NORTH: target_orient = Orientation.WEST
		elif robot.orientation == Orientation.SOUTH: target_orient = Orientation.EAST
		elif robot.orientation == Orientation.EAST: target_orient = Orientation.NORTH
		elif robot.orientation == Orientation.WEST: target_orient = Orientation.SOUTH
	elif dir == Direction.RIGHT:
		if robot.orientation == Orientation.NORTH: target_orient = Orientation.EAST
		elif robot.orientation == Orientation.SOUTH: target_orient = Orientation.WEST
		elif robot.orientation == Orientation.EAST: target_orient = Orientation.SOUTH
		elif robot.orientation == Orientation.WEST: target_orient = Orientation.NORTH
	elif dir == Direction.BACKWARD:
		# backwards logic, note that going backwards doesn't actually change the robots orientation
		if robot.orientation == Orientation.NORTH: target_orient = Orientation.SOUTH
		elif robot.orientation == Orientation.SOUTH: target_orient = Orientation.NORTH
		elif robot.orientation == Orientation.EAST: target_orient = Orientation.WEST
		elif robot.orientation == Orientation.WEST: target_orient = Orientation.EAST

	# Calculate coordinates based on absolute orientation
	if target_orient == Orientation.NORTH: ny += 1
	elif target_orient == Orientation.SOUTH: ny -= 1
	elif target_orient == Orientation.EAST: nx += 1
	elif target_orient == Orientation.WEST: nx -= 1

	return [nx, ny]



def is_valid(coords: List[int]):
	"""Check if the maze coords are within bounds"""
	return 0 <= coords[0] < 5 and 0 <= coords[1] < 5

def DFS():
	adjacent = []
	robot.orientation = Orientation.NORTH
	log("Running micromouse algorithm")

	while True:
		# Observe the walls
		currentPos = MAZE[currentY][currentX]
		# Add adjacents to stack
		if not API.wallFront():
			coords = get_neighbor_coords(Direction.FORWARD)
			if is_valid(coords):
				forward = MAZE[coords[0]][coords[1]]
				forward.direction = Direction.FORWARD
				adjacent.append(forward)
				log("Adding forward cell to adjacent queue")
		else:
			# if there is a wall, mark it
			currentPos.walls.add(get_side_from_direction(Direction.FORWARD))
		if not API.wallLeft():
			coords = get_neighbor_coords(Direction.LEFT)
			if is_valid(coords):
				left = MAZE[coords[0]][coords[1]]
				left.direction = Direction.LEFT
				adjacent.append(left)
				log("Adding left cell to adjacent queue")
		else:
			currentPos.walls.add(get_side_from_direction(Direction.LEFT))
		if not API.wallRight():
			coords = get_neighbor_coords(Direction.RIGHT)
			if is_valid(coords):
				right = MAZE[coords[0]][coords[1]]
				right.direction = Direction.RIGHT
				adjacent.append(right)
				log("Adding right cell to adjacent queue")
		else:
			currentPos.walls.add(get_side_from_direction(Direction.RIGHT))
		if API.wallBack():
			currentPos.walls.add(get_side_from_direction(Direction.BACKWARD))
		# move to adjacent cells
		next:Cell = adjacent.pop()
		log(f"Next Cell:\n{next}")
		moveDirection(next.direction)
		next.visited = True

		# If we've reached a dead end.
		if API.wallFront() and API.wallLeft() and API.wallRight():
			previous:Cell = adjacent.pop()
			log(f"REACHED DEAD END: Backtracking by going {previous.direction}")
			API.setColor(currentX, currentY, "r")
			while API.wallFront():
				current_orientation = robot.orientation
				API.turnLeft()

				if current_orientation == Orientation.NORTH: robot.orientation = Orientation.EAST
				elif current_orientation == Orientation.EAST: robot.orientation = Orientation.SOUTH
				elif current_orientation == Orientation.SOUTH: robot.orientation = Orientation.WEST
				elif current_orientation == Orientation.WEST: robot.orientation = Orientation.NORTH
			moveDirection(previous.direction)
	
		colors = ["g", "o","w", "y", "C"]
		API.setColor(currentX, currentY, colors[len(currentPos.walls)])
		log(f"Coordinates: X: {currentX}, Y: {currentY}")

			


if __name__ == "__main__":
	DFS()
	while True:
		if not API.wallFront():
			API.moveForward()

