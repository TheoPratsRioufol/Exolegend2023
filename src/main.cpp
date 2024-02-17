//#include "gladiator.h"
#include "Utils/traj.h"

#define THRESHOLD 0.1


int length = 2;
int count = length-1;
Coor arr[80]; //= {Coor{0, 2}, Coor{3, 3}};

Gladiator* gladiator;

float kw = 1.2;
float kv = 1.f;
float wlimit = 3.f;
float vlimit = 0.3f;
float erreurPos = 0.07;

float squareSize;

bool visited[12][12] = {};

enum SquareEdge{
    NORTH,    SOUTH,
    EAST,     WEST,
    CENTER
};

struct Coord{
    int i;
    int j;
};

double reductionAngle(double x)
{
    x = fmod(x + PI, 2 * PI);
    if (x < 0)
        x += 2 * PI;
    return x - PI;
}
void go_to(Position cons, Position pos)
{
    double consvl, consvr;
    double dx = cons.x - pos.x;
    double dy = cons.y - pos.y;
    double d = sqrt(dx * dx + dy * dy);

    if (d > erreurPos)
    {
        double rho = atan2(dy, dx);
        double consw = kw * reductionAngle(rho - pos.a);

        double consv = kv * d * cos(reductionAngle(rho - pos.a));
        consw = abs(consw) > wlimit ? (consw > 0 ? 1 : -1) * wlimit : consw;
        consv = abs(consv) > vlimit ? (consv > 0 ? 1 : -1) * vlimit : consv;

        consvl = consv - gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
        consvr = consv + gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
    }
    else
    {
        consvr = 0;
        consvl = 0;
    }

    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false); // GFA 3.2.1
    gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);  // GFA 3.2.1
}

Position goal {0.14f, 1.615f, 0.f};


Position getSquareCoor(const MazeSquare *square, SquareEdge edge = CENTER){
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
        coor.a = PI/2;
        break;
    case SOUTH:
        coor.x = (square->i + 0.5) * squareSize;
        coor.y = square->j * squareSize;
        coor.a = -PI/2;
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
    }

    return coor;
}

void reset() {
    //fonction de reset:
    //initialisation de toutes vos variables avant le début d'un match
    squareSize = gladiator->maze->getSquareSize(); //largeur d'une case GFA 4.7.4
    const MazeSquare *current_square = gladiator->maze->getNearestSquare();
        // const MazeSquare *target = gladiator->maze->getSquare(0, 0);
        // gladiator->log("Shortest path squa");
        gladiator->log("Start dij");

        hashMazeNode* mazeCosts = solve(current_square, gladiator);

        int itarget = random(0,12);
        int jtarget = random(0,12);

        mazeNode A;
        A.id = genId(current_square);
        mazeNode B;
        B.id = genId(itarget, jtarget);
        gladiator->log("Start Path");
        length = genPath(arr, mazeCosts, A, B,gladiator);

        gladiator->log("Disp Path");
        for (int k = 0; k < length; k++) {
            gladiator->log("CP %d = %d,%d",k,arr[k].i,arr[k].j);
        }
        goal = getSquareCoor(gladiator->maze->getSquare(arr[length-1].i, arr[length-1].j));
        count = length-1;
}

void setup() {
    //instanciation de l'objet gladiator
    gladiator = new Gladiator();
    //enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset); // GFA 4.4.1
}

float distance(const Position &p1, const Position &p2) {
    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}

Position current;
const MazeSquare *nearest_square;

bool checksquare(const MazeSquare *square){
    if(square != NULL && !visited[square->i][square->j]) {
        goal = getSquareCoor(square);
        visited[square->i][square->j] = true;
        return true;
    }

    return false;
}

void loop() {
    if(gladiator->game->isStarted()) { //tester si un match à déjà commencer
        //code de votre stratégie   
        current = gladiator->robot->getData().position;
        go_to(goal, current);
        if (distance(current, goal) <= THRESHOLD && count >= 0) {
            goal = getSquareCoor(gladiator->maze->getSquare(arr[count].i, arr[count].j));
            gladiator->log("Step %d l%d, Moving to %d,%d",count, length, arr[count].i, arr[count].j);
            count--;
        }
        delay(500);
    }
}

