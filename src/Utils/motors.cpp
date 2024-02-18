#include "Utils/motors.h"
// #include "Utils/graph.h"
#include <array>
// include min
#include <algorithm>

float kw = 5.0f;
float kv = 2.5f;
float wlimit = 4.f;
float vlimitMax = 1.6f;
float vlimitMin = 1.6f;
float erreurPos = 0.01;
float erreurPos_angle = 5 * DEG_TO_RAD;
float dRampe = 0;
float penteRampe = vlimitMax / dRampe;

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

double reductionAngle2(double x)
{
    x = fmod(x, 2 * PI);

    return x;
}

void go_to_no_u_turn(Position cons, Position pos, Gladiator *gladiator)
{
    double consvl, consvr;
    double dx = cons.x - pos.x;
    double dy = cons.y - pos.y;
    double d = sqrt(dx * dx + dy * dy);
    float vlimit = vlimitMax;

    if (d > erreurPos)
    {
        double rho = atan2(dy, dx);
        // double diff_angle = std::min(reductionAngle2(rho - pos.a), reductionAngle2(rho - pos.a + PI));
        double diff_angle;
        if(reductionAngle2(rho - pos.a)<HALF_PI || reductionAngle2(rho - pos.a)>3.0*HALF_PI){
            // log here
            gladiator->log("normal");
            diff_angle = reductionAngle(rho - pos.a);
            double consw = kw * diff_angle;

            if (d < dRampe)
            {
                if (d * penteRampe > vlimitMin)
                {
                    vlimit = d * penteRampe;
                }
                else
                {
                    vlimit = vlimitMin;
                }
            }

            double consv = kv * d * pow(cos(reductionAngle(rho - pos.a)), 15);
            consw = abs(consw) > wlimit ? (consw > 0 ? 1 : -1) * wlimit : consw;
            consv = abs(consv) > vlimit ? (consv > 0 ? 1 : -1) * vlimit : consv;

            consvl = consv - gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
            consvr = consv + gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
        }else{
            gladiator->log("reverse");
            diff_angle = reductionAngle(rho - pos.a-PI);
            double consw = -kw * diff_angle;
            if (d < dRampe)
            {
                if (d * penteRampe > vlimitMin)
                {
                    vlimit = d * penteRampe;
                }
                else
                {
                    vlimit = vlimitMin;
                }
            }

            double consv = kv * d * pow(cos(diff_angle), 15);

            consw = abs(consw) > wlimit ? (consw > 0 ? 1 : -1) * wlimit : consw;
            consv = abs(consv) > vlimit ? (consv > 0 ? 1 : -1) * vlimit : consv;

            consvl = consv + gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
            consvr = consv - gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
        }
        // double consw = kw * diff_angle;

        // double consv = kv * d * cos(diff_angle);
        // consw = abs(consw) > wlimit ? (consw > 0 ? 1 : -1) * wlimit : consw;
        // consv = abs(consv) > vlimit ? (consv > 0 ? 1 : -1) * vlimit : consv;
        // if(diff_angle==reductionAngle(rho - pos.a)){
        //     consvl = consv - gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
        //     consvr = consv + gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
        // }else{
        //     consvl = -consv - gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
        //     consvr = -consv + gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
        // }


        // gladiator->log("angle diff = %f",reductionAngle(rho - pos.a));
        // if (reductionAngle(rho - pos.a)> PI/12){
        //     consvl = -0.3 * reductionAngle(rho - pos.a)/PI;
        //     consvr = 0.3 * reductionAngle(rho - pos.a)/PI;
        // }
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

void go_to(Position cons, Position pos, Gladiator *gladiator)
{
    double consvl, consvr;
    double dx = cons.x - pos.x;
    double dy = cons.y - pos.y;
    double d = sqrt(dx * dx + dy * dy);
    float vlimit = vlimitMax;

    if (d > erreurPos)
    {
        double rho = atan2(dy, dx);
        double consw = kw * reductionAngle(rho - pos.a);

        if (d < dRampe)
        {
            if (d * penteRampe > vlimitMin)
            {
                vlimit = d * penteRampe;
            }
            else
            {
                vlimit = vlimitMin;
            }
        }

        double consv = kv * d * pow(cos(reductionAngle(rho - pos.a)), 15);
        consw = abs(consw) > wlimit ? (consw > 0 ? 1 : -1) * wlimit : consw;
        consv = abs(consv) > vlimit ? (consv > 0 ? 1 : -1) * vlimit : consv;

        consvl = consv - gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
        consvr = consv + gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2

        // gladiator->log("angle diff = %f",reductionAngle(rho - pos.a));
        // if (reductionAngle(rho - pos.a)> PI/12){
        //     consvl = -0.3 * reductionAngle(rho - pos.a)/PI;
        //     consvr = 0.3 * reductionAngle(rho - pos.a)/PI;
        // }
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

void motor_handleMvt(WayToGo *wayToGo, Gladiator *gladiator, int deleted, bool fireRocket, unsigned char robot_id, 
        States* myState_ptr, RocketMonitoring* rocketMonitoring)
{
    current = gladiator->robot->getData().position;
    // log *myState_ptr
    // gladiator->log("myState_ptr : %d", *myState_ptr);
    if(*myState_ptr == DEFENSE){
        // goto à une distance raisonnable dans une direction sans mur qui soit la plus perpendiculaire à la trajectoire de la fusée et qui prend en compte l'orientation du robot (go_to_no_u_turn)
        // puis tourne en rond jusqu'a ce que il ne soit plus aimed at

        // get adjacent squares
        const MazeSquare *neerestSquare = gladiator->maze->getNearestSquare();
        MazeSquare *adjacentSquares[4] = {neerestSquare->northSquare, neerestSquare->southSquare, neerestSquare->eastSquare, neerestSquare->westSquare};
        pos_2d adjacentSquare_pos[4];
        for(int i=0; i<4; i++){
            if(adjacentSquares[i] != nullptr){
                adjacentSquare_pos[i] = {3.0f/12.0f*adjacentSquares[i]->i, 3.0f/12.0f*adjacentSquares[i]->j};
            }else{
                adjacentSquare_pos[i] = {10, 10}; // big value so that it is eliminated
            }
        }
        // get the direction of the closest rocket that is aimed to us
        int rocket_index = -1;
        RobotData robot = gladiator->robot->getData();
        pos_2d robot_pos = {robot.position.x, robot.position.y};
        double min_distance = 10;
        for(int i=0;i<16;i++){
            if (rocketMonitoring->Rockets_on_map[i])
            {
                // check if rocket is going toward robot
                pos_2d rocket_dir = {rocketMonitoring->Rockets_on_map_end_pos[i].i - rocketMonitoring->Rockets_on_map_start_pos[i].i, rocketMonitoring->Rockets_on_map_end_pos[i].j - rocketMonitoring->Rockets_on_map_start_pos[i].j};
                // rocket goes in a straith line at a speed of ROCKET_SPEED m/s
                double angle_rocket_path = atan2(rocket_dir.j, rocket_dir.i);
                pos_2d rocket_now = {rocketMonitoring->Rockets_on_map_start_pos[i].i + ROCKET_SPEED * cos(angle_rocket_path), rocketMonitoring->Rockets_on_map_start_pos[i].j + ROCKET_SPEED * sin(angle_rocket_path)};

                // calculatethe scalar product between the rocket direction and the vector robot_pos - rocket_now
                pos_2d robot_to_rocket = {robot_pos.i - rocket_now.i, robot_pos.j - rocket_now.j};
                double scalar_product = rocket_dir.i * robot_to_rocket.i + rocket_dir.j * robot_to_rocket.j;
                if (scalar_product < 0)
                {
                    continue;
                }

                // check if distance to rocket path is small
                Segment rocket_path = {rocketMonitoring->Rockets_on_map_start_pos[i], rocketMonitoring->Rockets_on_map_end_pos[i]};
                double dist = distancePointToSegment(robot_pos, rocket_path);
                if (dist < min_distance)
                {
                    min_distance = dist;
                    rocket_index = i;
                }
            }
        }
        if(rocket_index == -1){
            *myState_ptr = EAT_AS_POSSIBLE;
            return;
        }
        // get the direction of the rocket
        pos_2d rocket_dir = {rocketMonitoring->Rockets_on_map_end_pos[rocket_index].i - rocketMonitoring->Rockets_on_map_start_pos[rocket_index].i, rocketMonitoring->Rockets_on_map_end_pos[rocket_index].j - rocketMonitoring->Rockets_on_map_start_pos[rocket_index].j};


        // get scalar product of the direction of the rocket and the direction of the robot according to the adjacent squares
        double scalar_product[4];
        for(int i=0; i<4; i++){
            scalar_product[i] = rocket_dir.i * (adjacentSquare_pos[i].i - robot_pos.i) + rocket_dir.j * (adjacentSquare_pos[i].j - robot_pos.j);
        }
        // get the index of the adjacent square that is the most perpendiculare to the rocket path (the one with the smallest scalar product)
        int index = 0;
        double min_scalar_product = 100;
        for(int i=0; i<4; i++){
            if(scalar_product[i] < min_scalar_product){
                min_scalar_product = scalar_product[i];
                index = i;
            }
        }
        // get the position of the adjacent square that is the most perpendiculare to the rocket path
        pos_2d goal_pos = adjacentSquare_pos[index];
        // get the direction to that square
        pos_2d dir = {goal_pos.i - robot_pos.i, goal_pos.j - robot_pos.j};
        // advance by 1*3/12 in that direction
        pos_2d goal_pos_2 = {robot_pos.i + 1.0*3.0/12.0*dir.i, robot_pos.j + 1.0*3.0/12.0*dir.j};
        // log goal_pos_2
        gladiator->log("goal_pos_2 : %f, %f", goal_pos_2.i, goal_pos_2.j);
        
        float angle_fin = (dir.i == 0) ? 0 : HALF_PI;
        go_to_no_u_turn({goal_pos_2.i, goal_pos_2.j, angle_fin}, robot.position, gladiator);

        // if the goal is reached, turn around
        if (distance(robot.position, {goal_pos_2.i, goal_pos_2.j, angle_fin}) < erreurPos)
        {
            double angle_robot = robot.position.a;
            go_to_angle(angle_robot + PI-5.0*DEG_TO_RAD, robot.position.a, gladiator);
        }


    }else{
        if(fireRocket || (*myState_ptr == ATTACK && (distance(current, goal) <= THRESHOLD) && !wayToGo->hasFinish())){
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
            // const MazeSquare* neerestSquare_enemy = gladiator->maze->getSquare(float(pos_other.x*12.0/3.0), float(pos_other.y*12.0/3.0));
            float vl_other = robot_other.vl;
            float vr_other = robot_other.vr;

            //position next second if vl and vr constant
            float time_future = 2;
            Position pos_other_future = {cos(pos_other.a) * (vl_other + vr_other) / 2 * time_future + pos_other.x, sin(pos_other.a) * (vl_other + vr_other) / 2 * time_future + pos_other.y, pos_other.a + (vr_other - vl_other) / gladiator->robot->getRobotRadius() * time_future};

            double rho = atan2(pos_other_future.y - pos.y, pos_other_future.x - pos.x);




            go_to_angle(rho, pos.a, gladiator);
            if (abs(reductionAngle(rho - pos.a)) < 1*DEG_TO_RAD) {
                if(*myState_ptr!=ATTACK){
                    gladiator->weapon->launchRocket();
                }else{
                    *myState_ptr = EAT_AS_POSSIBLE;
                }

            }
        }else{
            go_to(goal, current, gladiator); //FORGET
            // go_to_no_u_turn(goal, current, gladiator);
        }
        // gladiator->log("Goal=%f,%f cur=%d,%d c %d", goal.x, goal.y, gladiator->maze->getNearestSquare()->i, gladiator->maze->getNearestSquare()->j, count);
        if ((distance(current, goal) <= THRESHOLD) && !wayToGo->hasFinish() && *myState_ptr!=ATTACK)
        {
            wayToGo->moveToNext();
            if (wayToGo->hasFinish())
            {
                return;
            }
            // gladiator->log("Next Goal = %d,%d", listPos[count - 1].i, listPos[count - 1].j);
            // goal = getSquareCoor(gladiator->maze->getSquare(listPos[count - 1].i, listPos[count - 1].j));
            gladiator->log("Next Goal = %d,%d L%d,C%d", wayToGo->getNext().i, wayToGo->getNext().j, wayToGo->getLengthSorted(), wayToGo->currentShorted_idx);
            goal = getSquareCoor(gladiator->maze->getSquare(wayToGo->getNext().i, wayToGo->getNext().j));
            

            // si on va dans le mur on change de start
            // if (isBoundarie(listPos[count - 1].i, listPos[count - 1].j, deleted) && (length - count - 1 > 1))
            // {
            //     return -1;
            // }
            /*
            if (isBoundarie(wayToGo->getNext().i, wayToGo->getNext().j, deleted) && (false))
            {
                return;
            }*/
        }
    }
}



unsigned char closestRobotEnemy(Gladiator *gladiator)
{
    Position pos = gladiator->robot->getData().position;
    RobotList robotList = gladiator->game->getPlayingRobotsId();
    unsigned char closestRobot = 0;
    float minDist = 10;
    // log RobotList
    // gladiator->log("RobotList : %d, %d, %d", robotList.ids[0], robotList.ids[1], robotList.ids[2], robotList.ids[3]);

    for (int i = 0; i < 4; i++)
    {
        if (gladiator->game->getOtherRobotData(robotList.ids[i]).lifes && gladiator->game->getOtherRobotData(robotList.ids[i]).position.x!=0.0)
        {
            // gladiator->log("ID de l'autre robot : %d", robotList.ids[i]);
            if (gladiator->game->getOtherRobotData(robotList.ids[i]).teamId != gladiator->robot->getData().teamId)
            {
                Position pos_other = gladiator->game->getOtherRobotData(robotList.ids[i]).position;
                float dist = distance(pos, pos_other);
                if (dist < minDist)
                {
                    minDist = dist;
                    closestRobot = robotList.ids[i];
                }
            }
        }
    }
    // gladiator->log("got closest robot : %d", closestRobot);
    return closestRobot;
}

// std::array<unsigned char, 2> slowed_down_enemies(Gladiator *gladiator) {
//     std::array<unsigned char, 2> enemy_ids = {0, 0};
//     int count = 0; // Counter to keep track of how many slowed down enemies found
//     // check if one of the enemy robots is slowed down by a wall
//     RobotList robotList = gladiator->game->getPlayingRobotsId();
//     for (int i = 0; i < 4; i++) {
//         gladiator->log("here2");
//         if (gladiator->game->getOtherRobotData(robotList.ids[i]).lifes &&
//             gladiator->game->getOtherRobotData(robotList.ids[i]).position.x != 0.0) {
//             if (gladiator->game->getOtherRobotData(robotList.ids[i]).teamId !=
//                 gladiator->robot->getData().teamId) {
//                 if (gladiator->game->getOtherRobotData(robotList.ids[i]).speedLimit < 0.2) {
//                     enemy_ids[count++] = robotList.ids[i];
//                     if (count == 2) // Break if already found two slowed down enemies
//                         break;
//                 }
//             }
//         }
//     }
//     return enemy_ids;
// }

// std::array<unsigned char, 2> is_reachable_enemy(Gladiator *gladiator, hashMazeNode *mazeCosts){
//     RobotList robotList = gladiator->game->getPlayingRobotsId();
//     std::array<unsigned char, 2> enemy_ids = {0, 0};
//     int count = 0; // Counter to keep track of how many reachable enemies found
//     // check if one of the enemy robots is slowed down by a wall
//     for (int i = 0; i < 4; i++) {
//         gladiator->log("here3");
//         robot = gladiator->game->getOtherRobotData(robotList.ids[i]);
//         if (robot.lifes && robot.position.x != 0.0) {
//             if (robot.teamId != gladiator->robot->getData().teamId) {
//                 gladiator->log("here4");
                
//                 my_robot.square = (MazeSquare*)gladiator->maze->getNearestSquare();
//                 gladiator->log("my_robot.square : %d, %d", my_robot.square->i, my_robot.square->j);
//                 if (my_robot.square == nullptr) {
//                     // Handle null pointer
//                     gladiator->log("my_robot.square is nullptr");
//                     continue;
//                 }
//                 my_robot.id = genId(my_robot.square->i, my_robot.square->j);
//                 gladiator->log("my_robot.id : %d", my_robot.id);
                
//                 other_robot.square = (MazeSquare*)gladiator->maze->getSquare(robot.position.x*12.0/3.0, robot.position.y*12.0/3.0);
//                 gladiator->log("other_robot.square : %d, %d", other_robot.square->i, other_robot.square->j);
//                 if (other_robot.square == nullptr) {
//                     // Handle null pointer
//                     gladiator->log("other_robot.square is nullptr");
//                     continue;
//                 }
//                 other_robot.id = genId(other_robot.square->i, other_robot.square->j);
//                 gladiator->log("other_robot.id : %d", other_robot.id);
                
//                 gladiator->log("here5");
                
//                 int LL = genPath(&zeroc, mazeCosts, my_robot, other_robot, gladiator);
//                 // print LL
//                 gladiator->log("LL : %d", LL);
//                 gladiator->log("here6");
//                 if (LL < 4) { // Check count before accessing enemy_ids
//                     gladiator->log("count : %d", count);
//                     enemy_ids[count++] = robotList.ids[i];
//                     if (count == 2) // Break if already found two reachable enemies
//                         break;
//                 }
//             }
//         }
//     }
//     gladiator->log("here8");
//     return enemy_ids;
// }




// std::array<unsigned char, 2> is_not_pointing_enemy(Gladiator *gladiator){
//     std::array<unsigned char, 2> enemy_ids = {0, 0};
//     int count = 0; // Counter to keep track of how many enemies not pointing towards us found
//     // check if one of the enemy robots is slowed down by a wall
//     RobotList robotList = gladiator->game->getPlayingRobotsId();
//     for (int i = 0; i < 4; i++) {
//         if (gladiator->game->getOtherRobotData(robotList.ids[i]).lifes &&
//             gladiator->game->getOtherRobotData(robotList.ids[i]).position.x != 0.0) {
//             if (gladiator->game->getOtherRobotData(robotList.ids[i]).teamId !=
//                 gladiator->robot->getData().teamId) {
//                 Position pos = gladiator->robot->getData().position;
//                 Position pos_other = gladiator->game->getOtherRobotData(robotList.ids[i]).position;
//                 double rho = atan2(pos_other.y - pos.y, pos_other.x - pos.x);
//                 if (abs(reductionAngle(rho - pos.a)) > 70*DEG_TO_RAD) {
//                     enemy_ids[count++] = robotList.ids[i];
//                     if (count == 2) // Break if already found two enemies not pointing towards us
//                         break;
//                 }
//             }
//         }
//     }
//     return enemy_ids;
// }

unsigned char is_vulnerable_enemy(Gladiator *gladiator, hashMazeNode *mazeCosts){
    // // check if one of the enemy robots is slowed down by a wall and is reachable and doesn't point towards us
    // gladiator->log("here2");
    // std::array<unsigned char, 2> slowed_down_enemies_ids = slowed_down_enemies(gladiator);
    // // log of the ids of the slowed down enemies
    // gladiator->log("Slowed down enemies ids : %d, %d", slowed_down_enemies_ids[0], slowed_down_enemies_ids[1]);
    // std::array<unsigned char, 2> reachable_enemies_ids = is_reachable_enemy(gladiator, mazeCosts);
    // // log of the ids of the reachable enemies
    // gladiator->log("Reachable enemies ids : %d, %d", reachable_enemies_ids[0], reachable_enemies_ids[1]);
    // std::array<unsigned char, 2> not_pointing_enemies_ids = is_not_pointing_enemy(gladiator);
    // // log of the ids of the enemies not pointing towards us
    // gladiator->log("Not pointing enemies ids : %d, %d", not_pointing_enemies_ids[0], not_pointing_enemies_ids[1]);
    mazeNode my_robot;
    mazeNode other_robot;
    RobotData robot;
    RobotList robotList = gladiator->game->getPlayingRobotsId();
    unsigned char enemy_id = 0;
    int count = 0; // Counter to keep track of how many reachable enemies found
    // check if one of the enemy robots is slowed down by a wall
    for (int i = 0; i < 4; i++) {
        robot = gladiator->game->getOtherRobotData(robotList.ids[i]);
        if (robot.lifes && robot.position.x != 0.0) {
            if (robot.teamId != gladiator->robot->getData().teamId) {
                
                my_robot.square = (MazeSquare*)gladiator->maze->getNearestSquare();
                if (my_robot.square == nullptr) {
                    // Handle null pointer
                    continue;
                }
                my_robot.id = genId(my_robot.square->i, my_robot.square->j);
                
                other_robot.square = (MazeSquare*)gladiator->maze->getSquare(robot.position.x*12.0/3.0, robot.position.y*12.0/3.0);
                if (other_robot.square == nullptr) {
                    // Handle null pointer
                    continue;
                }
                other_robot.id = genId(other_robot.square->i, other_robot.square->j);
                
                Position pos = gladiator->robot->getData().position;
                Position pos_other = robot.position;
                double rho = atan2(pos_other.y - pos.y, pos_other.x - pos.x);
                if(robot.speedLimit < 0.2 && abs(reductionAngle(rho - pos.a)) > 70*DEG_TO_RAD){
                    int LL = genPath(mazeCosts, my_robot, other_robot, gladiator);
                    // print LL
                    gladiator->log("LL : %d", LL);
                    if (LL < 6) { // Check count before accessing enemy_ids
                        gladiator->log("count : %d", count);
                        enemy_id = robotList.ids[i];
                        return enemy_id;
                    }
                }
            }
        }
    }
    
    return 0;
}

bool should_launch_rocket(Gladiator *gladiator, unsigned char robot_id_to_fire){
    // check if can laucnh rocket and if distance next second is within 4 tiles
    if(!gladiator->weapon->canLaunchRocket()){
        return false;
    }

    RobotData robot_other = gladiator->game->getOtherRobotData(robot_id_to_fire);
    Position pos_other = robot_other.position;
    Position pos = gladiator->robot->getData().position;

    // predict future position of the robot according to nearest walls and the robot's speed
    // const MazeSquare* neerestSquare_enemy = gladiator->maze->getSquare(float(pos_other.x*12.0/3.0), float(pos_other.y*12.0/3.0));
    float vl_other = robot_other.vl;
    float vr_other = robot_other.vr;

    //position next second if vl and vr constant
    float time_future = 1;
    Position pos_other_future = {cos(pos_other.a) * (vl_other + vr_other) / 2 * time_future + pos_other.x, sin(pos_other.a) * (vl_other + vr_other) / 2 * time_future + pos_other.y, pos_other.a + (vr_other - vl_other) / gladiator->robot->getRobotRadius() * time_future};
    
    // get distance between the two robots
    float dist = distance(pos, pos_other_future) * 12.0/3.0;
    return dist<=4;


}