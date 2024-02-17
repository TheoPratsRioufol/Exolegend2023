// #include "gladiator.h"

// #include "Utils/graph.h"
#include "Utils/motors.h"
#include "Utils/RocketMonitoring.h"

#define TIME_SKRINK 20000
#define TIME_ESCAPE_BOUND 12000
#define LEN_PATH_STRAT 3

#define TIME_SKRINK 15000
#define LEN_PATH_STRAT 2

Gladiator *gladiator;
RocketMonitoring *rocketMonitoring;

unsigned long timer = 0;
unsigned long dateOfLastShrink = 0;
int deleted = 1;
States myState = DEFAULT_STATE;

SimpleCoord arr[SIZE_MAX_WAY]; //= {Coor{0, 2}, Coor{3, 3}};
// SimpleCoord arrShorted[80]; //= {Coor{0, 2}, Coor{3, 3}};

WayToGo wayToGo;

bool start = true;

// Gladiator* gladiator;

void getDirStack()
{

    const MazeSquare *current_square = gladiator->maze->getNearestSquare();
    gladiator->log("Get New Strategy ================");
    hashMazeNode *mazeCosts = solve(current_square, gladiator, LEN_PATH_STRAT, deleted, myState); // assure que l'on est en dessous du critère

    // on cherche le meilleur candidat qui minimise le cout et respecte la target
    int bestTarget = genId(current_square);
    int minCost = MAX_COST;
    int minCheminsNb = 1000;
    int maxCriteria = 0;

    /*if ((myState == ESCAPE_BOUND))
    {
        gladiator->log("target is CENTER");
        for (int k = 0; k < MAZE_NUMBER_CELLS; k++)
        {
            mazeNode *candidate = mazeCosts->get(k);
            if (!isBoundarie(mazeCosts->get(k)->square->i, mazeCosts->get(k)->square->j, deleted) && (candidate->cost < minCost))
            {
                gladiator->log("get better cost than %d", minCost);
                bestTarget = k;
                minCost = candidate->cost;
                // minCheminsNb = candidate->cheminsNb;
            }
        }
    }
    else
    {*/
    for (int k = 0; k < MAZE_NUMBER_CELLS; k++)
    {
        mazeNode *candidate = mazeCosts->get(k);
        if ((candidate->stopCriteria > maxCriteria) && (candidate->stopCriteria <= LEN_PATH_STRAT))
        {
            maxCriteria = candidate->stopCriteria;
            bestTarget = k;
            minCost = candidate->cost;
        }
        else if ((candidate->stopCriteria == maxCriteria) && (candidate->cost < minCost))
        {
            minCost = candidate->cost;
            bestTarget = k;
        }
    }
    //}

    // on génère le path correspondant
    mazeNode A;
    A.id = genId(current_square);
    mazeNode B;
    B.id = bestTarget;

    int length = genPath(arr, mazeCosts, A, B, gladiator);

    gladiator->log("Choosen Path, stopCriteria = %d Min chemin%d", maxCriteria, minCheminsNb);
    for (int k = 0; k < length; k++)
    {
        gladiator->log("CP %d(%d) = %d,%d", k, genId(arr[k].i, arr[k].j), arr[k].i, arr[k].j);
    }

    if (length > 40)
    {
        B.id = B.id - 1;
        length = genPath(arr, mazeCosts, A, B, gladiator);
    }

    gladiator->log("Goal (trg=%d) cap=%d criteria=%d", bestTarget, (mazeCosts->get(bestTarget)->square->possession == gladiator->robot->getData().teamId), mazeCosts->get(bestTarget)->stopCriteria);
    gladiator->log("%d : cap=%d by maze", bestTarget, (gladiator->maze->getSquare(geti(bestTarget), getj(bestTarget))->possession == gladiator->robot->getData().teamId));

    wayToGo.pushArr(arr, length);

    // Contracter l'array des coordonnées à parcourir en arrShorted :
    wayToGo.simplify();
}

void reset()
{
    // fonction de reset:
    const MazeSquare *current_square = gladiator->maze->getNearestSquare();
    float size = gladiator->maze->getSquareSize();
    gladiator->log("Square size = %f/%f", gladiator->maze->getSquareSize(), size);
    reset_motors(current_square, size, gladiator);

    getDirStack();
    dateOfLastShrink = millis();

    rocketMonitoring = new RocketMonitoring();

    gladiator->log("ResetDone");
}

void lookWatch()
{
    if ((myState != ESCAPE_BOUND) && (millis() - dateOfLastShrink > TIME_ESCAPE_BOUND))
    {
        myState = ESCAPE_BOUND;
        gladiator->log("Mode ESCAPE_BOUND");
        getDirStack();
    }
    if (millis() - dateOfLastShrink > TIME_SKRINK)
    {
        deleted++;
        dateOfLastShrink = millis();
        myState = EAT_AS_POSSIBLE;
        getDirStack();
        gladiator->log("Mode EAT_AS_POSSIBLE");
    }
}

void setup()
{
    // instanciation de l'objet gladiator
    gladiator = new Gladiator();
    // enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset); // GFA 4.4.1
}
unsigned char robot_id_to_fire = 0;
void loop()
{
    if (gladiator->game->isStarted())
    { // tester si un match à déjà commencer
        // code de votre stratégie

        // print info about RocketMonitoring
        rocketMonitoring->monitoring_loop(gladiator);
        rocketMonitoring->print_info(gladiator);

        if (wayToGo.hasFinish())
        {
            gladiator->log("Finish path, starting new one from %d,%d", geti(genId(gladiator->maze->getNearestSquare())), getj(genId(gladiator->maze->getNearestSquare())));
            getDirStack();
        }
        if (gladiator->weapon->canLaunchRocket())
        {
            robot_id_to_fire = closestRobotEnemy(gladiator);
            motor_handleMvt(&wayToGo, gladiator, deleted);
        }
        else
        {
            motor_handleMvt(&wayToGo, gladiator, deleted);
        }
        lookWatch();
        delay(10);
    }
}