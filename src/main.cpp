// #include "gladiator.h"
// Gladiator *gladiator;
// void reset();
// void setup()
// {
//     // instanciation de l'objet gladiator
//     gladiator = new Gladiator();
//     // enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
//     gladiator->game->onReset(&reset); // GFA 4.4.1

//     // gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0.5, false);
// }

// void reset()
// {
//     // fonction de reset:
//     // initialisation de toutes vos variables avant le début d'un match
//     gladiator->log("Call of reset function"); // GFA 4.5.1
// }

// void loop()
// {
//     gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0.5, false);
//     if (gladiator->game->isStarted())
//     { // tester si un match à déjà commencer
//         // code de votre stratégie
//         gladiator->log("Hello world - Game Started"); // GFA 4.5.1
//     }
//     else
//     {
//         gladiator->log("Hello world - Game not Startd yet"); // GFA 4.5.1
//     }
//     delay(300);
// }


#include "gladiator.h"
Gladiator* gladiator;

float kw = 1.2;
float kv = 1.f;
float wlimit = 3.f;
float vlimit = 0.6;
float erreurPos = 0.07;

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



void reset();
void setup() {
    //instanciation de l'objet gladiator
    gladiator = new Gladiator();
    //enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset); // GFA 4.4.1

    gladiator->log( "x = %f", gladiator->robot->getData().position.x);
    gladiator->log( "y = %f", gladiator->robot->getData().position.y);
    gladiator->log( "a = %f", gladiator->robot->getData().position.a);
}

void reset() {
    //fonction de reset:
    //initialisation de toutes vos variables avant le début d'un match
    gladiator->log("Call of reset function"); // GFA 4.5.1
}

void loop() {
    if(gladiator->game->isStarted()) { //tester si un match à déjà commencer
        //code de votre stratégie
        // Position myPosition = gladiator->robot->getData().position;
        // Position goal {0.5,1.5,0};
        // go_to(goal, myPosition);

        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0.5, false);
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, 0.5, false);
        delay(1000);
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0, false);
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, 0, false);
        delay(1000);
    }
    delay(10);
}