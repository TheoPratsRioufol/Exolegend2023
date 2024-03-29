#include "Utils/motors.h"

float kw = 5.0f;
float kv = 1.5f;
float wlimit = 5.f;
float vlimitMax = .6f;
float vlimitMin = .1f;
float erreurPos = 0.01;
float dRampe = 0.5;

float squareSize;

bool visited[12][12] = {};
Position goal{0.14f, 1.615f, 0.f};
Position current;
const MazeSquare *nearest_square;

Position getSquareCoor(const MazeSquare *square, SquareEdge edge)
{
    Position coor;
    // pour calculer les coordonnées x et y il faut récupérer les index i et j de la case
    switch (edge)
    {
    case CENTER:
        coor.x = (square->i + 0.5) * squareSize;
        coor.y = (square->j + 0.5) * squareSize;
        coor.a = 0;
        break;
    case NORTH:
        coor.x = (square->i + 0.5) * squareSize;
        coor.y = (square->j + 1) * squareSize;
        coor.a = PI / 2;
        break;
    case SOUTH:
        coor.x = (square->i + 0.5) * squareSize;
        coor.y = square->j * squareSize;
        coor.a = -PI / 2;
        break;
    case EAST:
        coor.x = (square->i + 1) * squareSize;
        coor.y = (square->j + 0.5) * squareSize;
        coor.a = 0;
        break;
    case WEST:
        coor.x = square->i * squareSize;
        coor.y = (square->j + 0.5) * squareSize;
        coor.a = 0;
        break;
    default:
        coor.x = (square->i + 0.5) * squareSize;
        coor.y = (square->j + 0.5) * squareSize;
        coor.a = 0;
        break;
    }

    return coor;
}

void reset_motors(const MazeSquare *firstSquare, float squareSize_, Gladiator *gladiator)
{
    squareSize = squareSize_;
    goal = getSquareCoor(firstSquare);
    for (int i = 0; i < 12; i++)
    {
        for (int j = 0; j < 12; j++)
        {
            visited[i][j] = false;
        }
    }
    // gladiator->log("RESET SZ = %f",squareSize_);
    // gladiator->log("RESET Goal = %f,%f vs %d,%d",goal.x,goal.y,firstSquare->i,firstSquare->j);
}

double reductionAngle(double x)
{
    x = fmod(x + PI, 2 * PI);
    if (x < 0)
        x += 2 * PI;
    return x - PI;
}
void go_to(Position cons, Position pos, Gladiator *gladiator)
{
    double consvl, consvr;
    double dx = cons.x - pos.x;
    double dy = cons.y - pos.y;
    double d = sqrt(dx * dx + dy * dy);

    if (d > erreurPos)
    {
        double rho = atan2(dy, dx);
        double consw = kw * reductionAngle(rho - pos.a);

        if (d < dRampe) {
            float vlimit = max;
        } else {
            float vlimit = vlimitMax;
        }
        
        double consv = kv * d * cos(reductionAngle(rho - pos.a));
        consw = abs(consw) > wlimit ? (consw > 0 ? 1 : -1) * wlimit : consw;
        consv = abs(consv) > vlimit ? (consv > 0 ? 1 : -1) * vlimit : consv;

        consvl = consv - gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
        consvr = consv + gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2

        //gladiator->log("angle diff = %f",reductionAngle(rho - pos.a));
        /*
        if (reductionAngle(rho - pos.a)> PI/12){
            consvl = -0.3 * reductionAngle(rho - pos.a)/PI;
            consvr = 0.3 * reductionAngle(rho - pos.a)/PI;
        }*/
    }
    else
    {
        consvr = 0;
        consvl = 0;
    }

    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false); // GFA 3.2.1
    gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);  // GFA 3.2.1
}

float distance(const Position &p1, const Position &p2)
{
    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}

bool checksquare(const MazeSquare *square)
{
    if (square != NULL && !visited[square->i][square->j])
    {
        goal = getSquareCoor(square);
        visited[square->i][square->j] = true;
        return true;
    }

    return false;
}

int motor_handleMvt(SimpleCoord *listPos, int count, int length, Gladiator *gladiator, int deleted)
{
    current = gladiator->robot->getData().position;
    go_to(goal, current, gladiator);
    //gladiator->log("Goal=%f,%f cur=%d,%d c %d l%d", goal.x, goal.y, gladiator->maze->getNearestSquare()->i, gladiator->maze->getNearestSquare()->j, count,length);
    if (distance(current, goal) <= THRESHOLD && count < length)
    {
        if (count + 1 >= length)
        {
            return -1;
        }
        gladiator->log("Next Goal = %d,%d", listPos[count + 1].i, listPos[count + 1].j);
        goal = getSquareCoor(gladiator->maze->getSquare(listPos[count + 1].i, listPos[count + 1].j));
        // si on va dans le mur on change de start
        if (isBoundarie(listPos[count + 1].i, listPos[count + 1].j, deleted) && (count > 100000000))
        {
            return -1;
        }
        if (count + 1 >= length)
        {
            return -1;
        }
        return count+1;
    }
    return count;
}