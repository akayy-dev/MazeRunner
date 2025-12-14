enum RobotOrientation {
  NORTH = 1,
  EAST = 2,
  SOUTH = 3,
  WEST = 4
};


struct Position {
  int xPos;
  int yPos;
};

struct Cell {
  // Whether or not the cells are touching a wall.
  bool northWall;
  bool southWall;
  bool eastWall;
  bool westWall;
  uint8_t distance;

  // Position of the cell, only keeping here for FlodFill
  Position pos;

  // Have we visited this cell before?
  bool visited;
};