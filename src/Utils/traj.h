#include "gladiator.h"

#define NB_ROW 16
#define MAZE_NUMBER_CELLS NB_ROW *NB_ROW
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
    mazeNode *elements[MAZE_NUMBER_CELLS];
    int last_idx = 0;
    void push(mazeNode node)
    {
        elements[last_idx] = &node;
        last_idx++;
    }
    mazeNode *pop()
    {
        last_idx--;
        return elements[last_idx];
    }
    int len()
    {
        return last_idx;
    }
    mazeNode *get(int idx)
    {
        return elements[idx];
    }
    boolean has(mazeNode node)
    {
        for (int i = 0; i < last_idx; i++)
        {
            if ((*(elements[i])).id == node.id)
                return true;
        }
        return false;
    }
    void print(Gladiator *glad)
    {
        /*
        for (int k = 0; k < last_idx; k++)
        {
            glad->log("blk = %d ", (*(elements[k])).square->i);
        }*/
    }
};

class hashMazeNode
{
public:
    mazeNode *elements[MAZE_NUMBER_CELLS];
    hashMazeNode()
    {
        for (int i = 0; i < MAZE_NUMBER_CELLS; i++)
        {
            mazeNode emptyNode;
            elements[i] = &emptyNode;
        }
    }
    void add(mazeNode node)
    {
        *(elements[node.id]) = node;
    }
    mazeNode *get(int id)
    {
        return elements[id];
    }
    bool has(mazeNode node)
    {
        return ((*(elements[node.id])).id != -1);
    }
};

int genId(const MazeSquare *start_);
void getNeighborS(mazeNode *workingNode_, listMazeNode *res, Gladiator *glad);
mazeNode extractMinCost(listMazeNode *frontier, Gladiator *glad);
int cost(mazeNode nodeA, mazeNode nodeB);
hashMazeNode *solve(const MazeSquare *start_, Gladiator *glad);