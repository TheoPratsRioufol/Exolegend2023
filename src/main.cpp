//#include "gladiator.h"

Gladiator* gladiator;

#include "Utils/traj.h"
#include "Utils/motors.h"

#define THRESHOLD 0.1


int length = 2;
int count = length-1;
Coor arr[80]; //= {Coor{0, 2}, Coor{3, 3}};

//Gladiator* gladiator;



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

