#ifndef GLADIATOR
#include "gladiator.h"
#endif

#include "Utils/traj.h"

listMazeNode neighbors;
hashMazeNode GlobalCost;



void getNeighborS(mazeNode *workingNode_, Gladiator *glad)
{
    //glad->log("nei 1");
   // glad->log("id = %d", workingNode_->id);
    //glad->log("blk i = %d", workingNode_->square->i);
    if (workingNode_->square->northSquare != nullptr)
    {
        mazeNode temp;
       // glad->log("if 1");
        temp.square = workingNode_->square->northSquare;
        temp.id = genId(temp.square);
        neighbors.push(temp);
    //glad->log("push id==%d loc = %d",temp.id,0);
    }
    //glad->log("nei 2");
    if (workingNode_->square->southSquare != nullptr)
    {
        mazeNode temp2;
        temp2.square = workingNode_->square->southSquare;
        temp2.id = genId(temp2.square);
        neighbors.push(temp2);
        //glad->log("push id==%d loc = %d",temp2.id,0);
    }
    //glad->log("nei 3");
    if (workingNode_->square->westSquare != nullptr)
    {
        mazeNode temp3;
        temp3.square = workingNode_->square->westSquare;
        temp3.id = genId(temp3.square);
        neighbors.push(temp3);
        //glad->log("push id==%d loc = %d",temp3.id,0);
    }
   // glad->log("nei 4");
    if (workingNode_->square->eastSquare != nullptr)
    {
        mazeNode temp4;
        temp4.square = workingNode_->square->eastSquare;
        temp4.id = genId(temp4.square);
        neighbors.push(temp4);
        //glad->log("push id==%d loc = %d",temp4.id,0);
    }
   // glad->log("nei 5");
}

mazeNode extractMinCost(listMazeNode *frontier, Gladiator *glad)
{
   // glad->log("extractMinCost 1 len = %d",frontier->len());
    // glad->log("extractMinCost l = %d", frontier.len());
    // glad->log("extractMinCost l = %d vs %d", frontier.pop(), nullptr);
    mazeNode retur = frontier->pop();
   // glad->log("extracted 1 len = %d",frontier->len());
    return retur;
    // glad->log("extractMinCost 2");
}

int cost(mazeNode nodeA, mazeNode nodeB)
{
    return 1;
}

int genId(const MazeSquare *start_)
{
    return start_->i * NB_ROW + start_->j;
}

int geti(int id) {
    return id/NB_ROW;
}

int getj(int id) {
    return id%NB_ROW;
}

hashMazeNode *solve(const MazeSquare *start_, Gladiator *glad)
{
    mazeNode start;
    start.square = (MazeSquare*) start_;
    start.id = genId(start_);
    listMazeNode frontier;
    //hashMazeNode *costs = &GlobalCost;
    //costs = *GlobalCost;
    GlobalCost.reset();
    frontier.push(start);
    glad->log("PUSH ptr = %d", frontier.get(0));
    GlobalCost.add(start);

    glad->log("call inside solve");

    mazeNode workingNode;
    mazeNode nextNode;
    //glad->log("l = %d", frontier.len());
    //glad->log("l = %d", neighbors.len());
    int niter = 0;

    while (frontier.len() > 0 && niter < 400)
    {
        niter++;
        // glad->log("l initial = %d", frontier.len());
        workingNode = extractMinCost(&frontier, glad);
        //glad->log("l after extract initial = %d", frontier.len());
        //glad->log("loop2 et reset and working = %d",workingNode.id);
        neighbors.last_idx = 0;
        getNeighborS(&workingNode, glad);
       // glad->log("COST OF WORK SOL = %d",workingNode.cost);
        //neighbors->print(glad);
       // glad->log("l = %d", frontier.len());

       // glad->log("nb of neibourgs = %d", neighbors.len());

        for (int i = 0; i < neighbors.len(); i++)
        {
           // glad->log("read neigborg %d of id = %d of %d", i, neighbors.get(i)->id, &neighbors.elements[i]);
            nextNode = *neighbors.get(i);
           // glad->log("next node = %d",nextNode.id);
            if (!GlobalCost.has(nextNode))
            {
                glad->log("add node = %d",nextNode.id);
                frontier.push(nextNode);
            }
            int newCost = GlobalCost.get(workingNode.id)->cost + cost(workingNode, nextNode);
            if (!GlobalCost.has(nextNode) || GlobalCost.get(nextNode.id)->cost > newCost)
            {
               // glad->log("COST BEFORE ADD has = %d with %d",costs->has(nextNode),costs->elements[nextNode.id].id);
                GlobalCost.add2(nextNode.id, workingNode, newCost, nextNode.square);
               // glad->log("COST ADD has = %d with %d",costs->has(nextNode),costs->elements[nextNode.id].id);
                glad->log("this %d parent %d, R parent %d",nextNode.id,workingNode.id,GlobalCost.get(nextNode.id)->parent);
            }
        }
    }
    glad->log("cend while");

    return &GlobalCost;
}


void printPath(hashMazeNode *costs, mazeNode A, mazeNode B, Gladiator *glad) {
    mazeNode *prevNode = &B;
    glad->log("Go to %d",B.id);
    for (int i = 0; i < MAZE_NUMBER_CELLS; i++) {
        int nextid = costs->get(prevNode->id)->parent;
        glad->log("Go %d, %d before ! (id=%d) %d",geti(nextid),getj(nextid),prevNode->id,nextid);
        if (nextid == A.id) {
            break;
        }
        prevNode = costs->get(nextid);
    }
}