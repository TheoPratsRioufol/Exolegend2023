// #include "gladiator.h"

// #include "Utils/graph.h"
#include "Utils/motors.h"

#define TIME_SKRINK 15000
#define LEN_PATH_STRAT 1

Gladiator *gladiator;

unsigned long timer = 0;
int length = 2;
int count = length - 1;
int deleted = 0;
SimpleCoord arr[80]; //= {Coor{0, 2}, Coor{3, 3}};
bool start = true;

// Gladiator* gladiator;

void getDirStack()
{

    const MazeSquare *current_square = gladiator->maze->getNearestSquare();
    gladiator->log("Get New Strategy ================");
    hashMazeNode *mazeCosts = solve(current_square, gladiator, LEN_PATH_STRAT, deleted); // assure que l'on est en dessous du critère

    // on cherche le meilleur candidat qui minimise le cout et respecte la target
    int bestTarget = genId(current_square);
    int minCost = mazeCosts->get(bestTarget)->cost;
    int maxCriteria = 0;
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

    // on génère le path correspondant
    mazeNode A;
    A.id = genId(current_square);
    mazeNode B;
    B.id = bestTarget;

    length = genPath(arr, mazeCosts, A, B, gladiator);
    count = length - 1;

    gladiator->log("Choosen Path, stopCriteria = %d", maxCriteria);
    for (int k = 0; k < length; k++)
    {
        gladiator->log("CP %d(%d) = %d,%d", k, genId(arr[k].i, arr[k].j), arr[k].i, arr[k].j);
    }

    gladiator->log("Goal (trg=%d) cap=%d criteria=%d", bestTarget, (mazeCosts->get(bestTarget)->square->possession == gladiator->robot->getData().teamId), mazeCosts->get(bestTarget)->stopCriteria);
    gladiator->log("%d : cap=%d by maze", bestTarget, (gladiator->maze->getSquare(geti(bestTarget), getj(bestTarget))->possession == gladiator->robot->getData().teamId));
}

void reset()
{
    // fonction de reset:
    const MazeSquare *current_square = gladiator->maze->getNearestSquare();
    float size = gladiator->maze->getSquareSize();
    gladiator->log("Square size = %f/%f", gladiator->maze->getSquareSize(), size);
    reset_motors(current_square, size, gladiator);

    getDirStack();
    timer = millis();

    gladiator->log("ResetDone");
}

void lookWatch()
{
    // deleted++;
    if ((deleted < 5) && (millis() > timer))
    {
        deleted++;
        timer = millis() + TIME_SKRINK;
        gladiator->log("ARENA SCRINK");
    }
}

void setup()
{
    // instanciation de l'objet gladiator
    gladiator = new Gladiator();
    // enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset); // GFA 4.4.1
}

void loop()
{
    if (gladiator->game->isStarted())
    { // tester si un match à déjà commencer
        // code de votre stratégie
        count = motor_handleMvt(arr, count, length, gladiator, deleted);
        lookWatch();

        if (count == -1)
        {
            gladiator->log("Finish path, starting new one from %d,%d", geti(genId(gladiator->maze->getNearestSquare())), getj(genId(gladiator->maze->getNearestSquare())));
            getDirStack();
        }

        delay(10);
    }
}
