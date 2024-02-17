
#ifndef GLADIATOR
#include "gladiator.h"
#endif

#include "Utils/graph.h"


#define THRESHOLD 0.1

enum SquareEdge{
    NORTH,    SOUTH,
    EAST,     WEST,
    CENTER
};


void reset_motors(const MazeSquare *firstSquare, float squareSize_, Gladiator *gladiator);
Position getSquareCoor(const MazeSquare *square, SquareEdge edge  = CENTER);
double reductionAngle(double x);
void go_to(Position cons, Position pos, Gladiator *gladiator);
bool checksquare(const MazeSquare *square);
int motor_handleMvt(SimpleCoord *listPos, int count, int length, Gladiator *gladiator, int deleted, bool fireRocket, unsigned char robot_id);
float distance(const Position &p1, const Position &p2);
unsigned char closestRobotEnemy(Gladiator *gladiator);