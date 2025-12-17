enum RobotOrientation {
  NORTH = 1,
  EAST = 2,
  SOUTH = 3,
  WEST = 4
};


// The side of the wall, from a top down perspective
enum Side {
	TOP = 1,
	LEFT = 2,
	RIGHT = 3,
	BOTTOM = 4,
};

enum Direction {
	D_FORWARD = 1,
	D_BACKWARD = 2,
	D_LEFT = 3,
	D_RIGHT = 4,
};


struct Position {
  int xPos;
  int yPos;
};

struct Cell {
  // Whether or not the cells are touching a wall.
  bool forwardWall;
  bool southWall;
  bool rightWall;
  bool leftWall;
  uint8_t distance;

  // Position of the cell, only keeping here for FlodFill
  Position pos;

  // Have we visited this cell before?
  bool visited;
};
