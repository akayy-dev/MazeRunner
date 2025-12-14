from dataclasses import dataclass, field
from typing import List, Set
from orientation import *


@dataclass
class Cell:
	visited: bool
	distance: int
	direction: Direction
	X: int
	Y: int
	walls: Set[Side] = field(default_factory=set)