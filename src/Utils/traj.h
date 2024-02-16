#include "gladiator.h"

#define MAZE_NUMBER_CELLS 16 * 16
#define MAX_COST 10000

struct mazeNode
{
    mazeNode *parent;
    int cost = 0;
    int id = -1;
    bool haveParent = false;
    const MazeSquare *square;
};

class listMazeNode
{
public:
    mazeNode elements[MAZE_NUMBER_CELLS];
    int last_idx = -1;
    void push(mazeNode node)
    {
        elements[last_idx++] = node;
    }
    mazeNode pop()
    {
        return elements[last_idx--];
    }
    int len()
    {
        return last_idx;
    }
    mazeNode get(int idx)
    {
        return elements[idx];
    }
    boolean has(mazeNode node)
    {
        for (int i = 0; i < last_idx; i++)
        {
            if (elements[i].id == node.id)
                return true;
        }
        return false;
    }
};

class hashMazeNode
{
public:
    mazeNode elements[MAZE_NUMBER_CELLS];
    hashMazeNode()
    {
        for (int i = 0; i < MAZE_NUMBER_CELLS; i++)
        {
            mazeNode emptyNode;
            elements[i] = emptyNode;
        }
    }
    void add(mazeNode node)
    {
        elements[node.id] = node;
    }
    mazeNode get(int id)
    {
        return elements[id];
    }
    boolean has(mazeNode node)
    {
        return elements[node.id].id != -1;
    }
};

listMazeNode getNeighborS(mazeNode workingNode_);
mazeNode extractMinCost(listMazeNode *frontier);
int cost(mazeNode nodeA, mazeNode nodeB);
hashMazeNode *solve(const MazeSquare *start_);