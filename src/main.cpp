// #include "gladiator.h"
#include "Utils/traj.h"

Gladiator *gladiator;
void reset();
void setup()
{
    // instanciation de l'objet gladiator
    gladiator = new Gladiator();
    // enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset); // GFA 4.4.1
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
        // gladiator->log("Shortest path squa");
        gladiator->log("COMPUTE START");
        hashMazeNode *mazeCosts = solve(current_square, gladiator);

        for (int i = 0; i < MAZE_NUMBER_CELLS; i++)
        {
            // gladiator->log("Shortest path square: %d, %d", i, mazeCosts->get(i).cost);
        }
        gladiator->log("END");
        while (1)
            ;
    }
    else
    {
        gladiator->log("Hello world - Game not Startd yet"); // GFA 4.5.1
        delay(300);
    }
    delay(40);
}