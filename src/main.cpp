#include "gladiator.h"
#include "Maze_utils.h"
Gladiator* gladiator;
void reset();
void setup() {
    //instanciation de l'objet gladiator
    gladiator = new Gladiator();
    //enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset); // GFA 4.4.1
}

void reset() {
    //fonction de reset:
    //initialisation de toutes vos variables avant le début d'un match
    gladiator->log("Call of reset function"); // GFA 4.5.1
}

void loop()
{
    if (gladiator->game->isStarted())
    { // tester si un match à déjà commencer
        // code de votre stratégie
        //gladiator->log("Hello world - Game Started"); // GFA 4.5.1
        const MazeSquare* current_square = gladiator->maze->getNearestSquare();
        gladiator->log("Current square: %d, %d", current_square->i, current_square->j);
        // get adjacent squares
        const MazeSquare* adjacent_squares[4] = {
            current_square->northSquare,
            current_square->southSquare,
            current_square->eastSquare,
            current_square->westSquare};
        for (const auto& adjacent_square : adjacent_squares)
        {
            if (adjacent_square != nullptr)
            {
                gladiator->log("Adjacent square: %d, %d", adjacent_square->i, adjacent_square->j);
            }
        }
        const MazeSquare* target = gladiator->maze->getSquare(0, 0);
        MazeSquare* shortest_path[144];
        uint8_t shortest_path_length = 0;
        get_shortest_path(gladiator, current_square, target, shortest_path, shortest_path_length);
        gladiator->log("Shortest path length: %d", shortest_path_length);
        for (uint8_t i = 0; i < shortest_path_length; i++)
        {
            gladiator->log("Shortest path square: %d, %d", shortest_path[i]->i, shortest_path[i]->j);
        }
        delay(2000);

    }
    else
    {
        gladiator->log("Hello world - Game not Startd yet"); // GFA 4.5.1
        delay(300);
    }
    //delay(300);
}