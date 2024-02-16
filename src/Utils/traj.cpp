#ifndef GLADIATOR
#include "gladiator.h"
#endif

#include "Utils/traj.h"

listMazeNode getNeighborS(mazeNode workingNode_)
{
    listMazeNode res;
    mazeNode temp;
    if (workingNode_.square->northSquare != nullptr)
    {
        temp.square = workingNode_.square->northSquare;
        res.push(temp);
    }
    if (workingNode_.square->southSquare != nullptr)
    {
        temp.square = workingNode_.square->southSquare;
        res.push(temp);
    }
    if (workingNode_.square->westSquare != nullptr)
    {
        temp.square = workingNode_.square->westSquare;
        res.push(temp);
    }
    if (workingNode_.square->eastSquare != nullptr)
    {
        temp.square = workingNode_.square->eastSquare;
        res.push(temp);
    }
    return res;
}

mazeNode extractMinCost(listMazeNode *frontier)
{
    return frontier->pop();
}

int cost(mazeNode nodeA, mazeNode nodeB)
{
    return 1;
}

hashMazeNode *solve(const MazeSquare *start_, Gladiator *glad)
{
    mazeNode start;
    start.square = start_;
    listMazeNode *frontier;
    frontier = new listMazeNode();
    hashMazeNode *costs;
    costs = new hashMazeNode();
    frontier->push(start);
    costs->add(start);

    glad->log("call");

    mazeNode workingNode;
    mazeNode nextNode;
    mazeNode newNodeUpdated;

    while (frontier->len() > 0)
    {
        workingNode = extractMinCost(frontier);
        listMazeNode neighbors = getNeighborS(workingNode);
        neighbors.print(glad);
        glad->log("l = %d", frontier->len());

        for (int i = 0; i < neighbors.len(); i++)
        {
            nextNode = neighbors.get(i);
            if (!costs->has(nextNode))
            {
                frontier->push(nextNode);
            }
            int newCost = workingNode.cost + cost(workingNode, nextNode);
            if (!costs->has(nextNode) || costs->get(nextNode.id).cost > newCost)
            {
                newNodeUpdated = costs->get(nextNode.id);
                newNodeUpdated.cost = newCost;
                newNodeUpdated.parent = &workingNode;
                costs->add(newNodeUpdated);
            }
        }
    }

    return costs;
}