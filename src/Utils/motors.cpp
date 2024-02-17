#include "Utils/motors.h"

float kw = 8.0f;
float kv = 1.2f;
float wlimit = 7.f;
float vlimitMax = .6f;
float vlimitMin = .3f;
float erreurPos = 0.01;
float erreurPos_angle = 5*DEG_TO_RAD;
float dRampe = 0.5;
float penteRampe = vlimitMax/dRampe;

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

        float vlimit = vlimitMax;

        if (d < dRampe) {
            if (d * penteRampe > vlimitMin){vlimit = d * penteRampe;} else {vlimit = vlimitMin;}
        }
        
        double consv = kv * d * pow(cos(reductionAngle(rho - pos.a)),15);
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

void go_to_angle(float cons_angle, float pos_angle, Gladiator *gladiator)
{
    double consvl, consvr;
    const float K = 0.5;
    float diff_angle = reductionAngle(cons_angle-pos_angle);
    if (abs(diff_angle) > erreurPos_angle)
    {
        // if(diff_angle > 0){
        //     consvl = -0.3;
        //     consvr = 0.3;
        // }else{
        //     consvl = 0.3;
        //     consvr = -0.3;
        // }
        consvl = - K * diff_angle;
        consvr = K * diff_angle;

        consvl = abs(consvl) > 0.3 ? (consvl > 0 ? 0.3 : -0.3) : consvl;
        consvr = abs(consvr) > 0.3 ? (consvr > 0 ? 0.3 : -0.3) : consvr;
    }
    else
    {
        consvr = 0;
        consvl = 0;
    }

    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false); // GFA 3.2.1
    gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);  // GFA 3.2.1
}

float distance(const Position &p1, const Position &p2) {
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

int motor_handleMvt(SimpleCoord *listPos, int count, int length, Gladiator *gladiator, int deleted, bool fireRocket, unsigned char robot_id)
{
    current = gladiator->robot->getData().position;
    if(fireRocket) {
        RobotData robot_other = gladiator->game->getOtherRobotData(robot_id);
        Position pos_other = robot_other.position;
        // log de la position de l'autre robot
        // log de l'id
        // gladiator->log("ID de l'autre robot : %d", robot_id);
        // gladiator->log("Position de l'autre robot : %f, %f", pos_other.x, pos_other.y);
        Position pos = gladiator->robot->getData().position;
        // double rho = atan2(pos_other.y - pos.y, pos_other.x - pos.x);
        // log de rho
        // gladiator->log("Rho : %f", rho);
        // Position target = {pos.x, pos.y, rho};

        // predict future position of the robot according to nearest walls and the robot's speed
        const MazeSquare* neerestSquare_enemy = gladiator->maze->getSquare(float(pos_other.x*12.0/3.0), float(pos_other.y*12.0/3.0));
        float vl_other = robot_other.vl;
        float vr_other = robot_other.vr;

        //position next second if vl and vr constant
        Position pos_other_future = {cos(pos_other.a) * (vl_other + vr_other) / 2 + pos_other.x, sin(pos_other.a) * (vl_other + vr_other) / 2 + pos_other.y, pos_other.a + (vr_other - vl_other) / gladiator->robot->getRobotRadius()};

        double rho = atan2(pos_other_future.y - pos.y, pos_other_future.x - pos.x);




        go_to_angle(rho, pos.a, gladiator);
        if (abs(reductionAngle(rho - pos.a)) < 1*DEG_TO_RAD) {
            gladiator->weapon->launchRocket();
        }
    }else{
        go_to(goal, current, gladiator);
    }
    // gladiator->log("Goal=%f,%f cur=%d,%d c %d", goal.x, goal.y, gladiator->maze->getNearestSquare()->i, gladiator->maze->getNearestSquare()->j, count);
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

unsigned char closestRobotEnemy(Gladiator *gladiator){
    Position pos = gladiator->robot->getData().position;
    RobotList robotList = gladiator->game->getPlayingRobotsId(); 
    unsigned char closestRobot = 0;
    float minDist = 10;
    // log RobotList
    // gladiator->log("RobotList : %d, %d, %d", robotList.ids[0], robotList.ids[1], robotList.ids[2], robotList.ids[3]);

    for (int i = 0; i < 4; i++) {
        if(gladiator->game->getOtherRobotData(robotList.ids[i]).lifes){
            // gladiator->log("ID de l'autre robot : %d", robotList.ids[i]);
            if (gladiator->game->getOtherRobotData(robotList.ids[i]).teamId != gladiator->robot->getData().teamId) {
                Position pos_other = gladiator->game->getOtherRobotData(robotList.ids[i]).position;
                float dist = distance(pos, pos_other);
                if (dist < minDist) {
                    minDist = dist;
                    closestRobot = robotList.ids[i];
                }
            }
        }
    }
    // gladiator->log("got closest robot : %d", closestRobot);
    return closestRobot;
}