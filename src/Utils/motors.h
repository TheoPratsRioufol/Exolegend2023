
#ifndef GLADIATOR
#include "gladiator.h"
#endif

#include "Utils/graph.h"
#include "Utils/RocketMonitoring.h"
#include <array>

#define THRESHOLD 0.1

enum SquareEdge
{
    NORTH,
    SOUTH,
    EAST,
    WEST,
    CENTER
};

void reset_motors(const MazeSquare *firstSquare, float squareSize_, Gladiator *gladiator);
Position getSquareCoor(const MazeSquare *square, SquareEdge edge = CENTER);
double reductionAngle(double x);
void go_to(Position cons, Position pos, Gladiator *gladiator);
void go_to_no_u_turn(Position cons, Position pos, Gladiator *gladiator);
bool checksquare(const MazeSquare *square);
void motor_handleMvt(WayToGo *wayToGo, Gladiator *gladiator, int deleted, bool fireRocket, unsigned char robot_id, States* myState_ptr, RocketMonitoring* rocketMonitoring);
float distance(const Position &p1, const Position &p2);
unsigned char closestRobotEnemy(Gladiator *gladiator);
// std::array<unsigned char, 2> slowed_down_enemies(Gladiator *gladiator);
unsigned char is_vulnerable_enemy(Gladiator *gladiator, hashMazeNode *mazeCosts);

bool should_launch_rocket(Gladiator *gladiator,unsigned char robot_id_to_fire);