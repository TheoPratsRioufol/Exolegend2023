#include "gladiator.h"

// #include "Utils/motors.h"
#include "Structs/mobile_base.h"

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







void loop() {
    if(gladiator->game->isStarted()) { //tester si un match à déjà commencé
        //code de votre stratégie
        gladiator->log("Hello world - Game Started"); // GFA 4.5.1

        while(true) {
            // Position target_pos;

            // Obtenir le carré le plus proche du robot dans le labyrinthe
            const MazeSquare* nearestSquare = gladiator->maze->getNearestSquare();
            float squareSize = gladiator->maze->getSquareSize();

            Position center = {0,0};

            if (nearestSquare->eastSquare != nullptr) {
                center.x = (nearestSquare->eastSquare->i + 0.5) * squareSize;
                center.y = (nearestSquare->eastSquare->j + 0.5) * squareSize;
            } else if (nearestSquare->southSquare != nullptr) {
                center.x = (nearestSquare->southSquare->i + 0.5) * squareSize;
                center.y = (nearestSquare->southSquare->j + 0.5) * squareSize;
            } else if (nearestSquare->westSquare != nullptr) {
                center.x = (nearestSquare->westSquare->i + 0.5) * squareSize;
                center.y = (nearestSquare->westSquare->j + 0.5) * squareSize;
            } else {
                center.x = (nearestSquare->northSquare->i + 0.5) * squareSize;
                center.y = (nearestSquare->northSquare->j + 0.5) * squareSize;
            }

            // target_pos = center;

            bool reached = false;
            while (!reached) {
                Position current_pos = gladiator->robot->getData().position;
                
                reached = go_to(center, current_pos, gladiator);
            }
        }
        
    }else {
        gladiator->log("Hello world - Game not Started yet"); // GFA 4.5.1
    }
    delay(300);
}