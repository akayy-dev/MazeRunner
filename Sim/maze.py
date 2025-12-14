from dataclasses import dataclass, field
from typing import List
from orientation import *


@dataclass
class Cell:
	visited: bool
	distance: int
	direction: Direction
	X: int
	Y: int
	walls: List[Direction] = field(default_factory=list)