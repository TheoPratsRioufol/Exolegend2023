// #include "gladiator.h"
#include "Utils/traj.h"

Gladiator *gladiator;
void reset();
void setup()
{
    // instanciation de l'objet gladiator
    gladiator = new Gladiator();
    // enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset);                 // GFA 4.4.1
    gladiator->game->enableFreeMode(RemoteMode::OFF); // GFA 4.3.2
}

void reset()
{
    // fonction de reset:
    // initialisation de toutes vos variables avant le début d'un match
    gladiator->log("Call of reset function"); // GFA 4.5.1
}

void loop()
{
    if (gladiator->game->isStarted())
    { // tester si un match à déjà commencer
        // code de votre stratégie
        // gladiator->log("Hello world - Game Started"); // GFA 4.5.1
        const MazeSquare *current_square = gladiator->maze->getNearestSquare();
        // const MazeSquare *target = gladiator->maze->getSquare(0, 0);

        hashMazeNode *mazeCosts = solve(current_square);

        for (uint8_t i = 0; i < MAZE_NUMBER_CELLS; i++)
        {
            gladiator->log("Shortest path square: %d, %d", i, mazeCosts->get(i).cost);
        }
        delay(1000);
    }
    else
    {
        gladiator->log("Hello world - Game not Startd yet"); // GFA 4.5.1
        delay(300);
    }
}