#ifndef GLADIATOR
#include "gladiator.h"
#endif

#include "Utils/traj.h"

void getNeighborS(mazeNode *workingNode_, listMazeNode *res, Gladiator *glad)
{
    mazeNode temp;
    glad->log("nei 1");
    glad->log("id = %d", workingNode_->id);
    glad->log("blk i = %d", workingNode_->square->i);
    if (workingNode_->square->northSquare != nullptr)
    {
        glad->log("if 1");
        temp.square = workingNode_->square->northSquare;
        temp.id = genId(temp.square);
        glad->log("if 2");
        res->push(temp);
    }
    glad->log("nei 2");
    if (workingNode_->square->southSquare != nullptr)
    {
        temp.square = workingNode_->square->southSquare;
        temp.id = genId(temp.square);
        res->push(temp);
    }
    glad->log("nei 3");
    if (workingNode_->square->westSquare != nullptr)
    {
        temp.square = workingNode_->square->westSquare;
        temp.id = genId(temp.square);
        res->push(temp);
    }
    glad->log("nei 4");
    if (workingNode_->square->eastSquare != nullptr)
    {
        temp.square = workingNode_->square->eastSquare;
        temp.id = genId(temp.square);
        res->push(temp);
    }
    glad->log("nei 5");
}

mazeNode extractMinCost(listMazeNode frontier, Gladiator *glad)
{
    glad->log("extractMinCost 1");
    // glad->log("extractMinCost l = %d", frontier.len());
    // glad->log("extractMinCost l = %d vs %d", frontier.pop(), nullptr);
    return *frontier.pop();
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

hashMazeNode *solve(const MazeSquare *start_, Gladiator *glad)
{
    mazeNode start;
    start.square = start_;
    start.id = genId(start_);
    listMazeNode frontier;
    hashMazeNode *costs;
    costs = new hashMazeNode();
    frontier.push(start);
    glad->log("PUSH ptr = %d", frontier.get(0));
    costs->add(start);

    glad->log("call inside solve");

    mazeNode workingNode;
    mazeNode nextNode;
    mazeNode newNodeUpdated;
    glad->log("l = %d", frontier.len());
    int niter = 0;

    while (frontier.len() > 0 && niter < 100)
    {
        niter++;
        glad->log("loop");
        workingNode = extractMinCost(frontier, glad);
        glad->log("loop2");
        listMazeNode *neighbors = new listMazeNode();
        getNeighborS(&workingNode, neighbors, glad);
        glad->log("loop3");
        neighbors->print(glad);
        glad->log("l = %d", frontier.len());

        glad->log("nb of neibourgs = %d", neighbors->len());

        for (int i = 0; i < neighbors->len(); i++)
        {
            glad->log("read neigborg of id = %d", neighbors->get(i)->id);
            nextNode = *neighbors->get(i);
            if (!costs->has(nextNode))
            {
                frontier.push(nextNode);
            }
            int newCost = workingNode.cost + cost(workingNode, nextNode);
            if (!costs->has(nextNode) || costs->get(nextNode.id)->cost > newCost)
            {
                newNodeUpdated = *(costs->get(nextNode.id));
                newNodeUpdated.cost = newCost;
                newNodeUpdated.parent = &workingNode;
                costs->add(newNodeUpdated);
            }
        }
    }
    glad->log("cend while");

    return costs;
}