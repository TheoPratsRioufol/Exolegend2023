#include "gladiator.h"

float kw = 1.2;
float kv = 1.f;
float wlimit = 3.f;
float vlimit = 0.3f;
float erreurPos = 0.07;

float squareSize;

bool visited[12][12] = {};
Position goal {0.14f, 1.615f, 0.f};

enum SquareEdge{
    NORTH,    SOUTH,
    EAST,     WEST,
    CENTER
};

struct Coord{
    int i;
    int j;
};


void reset();
Position getSquareCoor(const MazeSquare *square, SquareEdge edge = CENTER);
double reductionAngle(double x);
void go_to(Position cons, Position pos);
bool checksquare(const MazeSquare *square);