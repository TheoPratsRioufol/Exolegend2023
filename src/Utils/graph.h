
#include "gladiator.h"

#define NB_ROW 12
#define MAZE_NUMBER_CELLS NB_ROW *NB_ROW
#define MAX_COST 10000
#define SIZE_MAX_WAY 80

#define DEFAULT_STATE EAT_AS_POSSIBLE

enum States
{
    EAT_AS_POSSIBLE,
    ESCAPE_BOUND,
    ATTACK,
    DEFENSE
};

struct SimpleCoord
{
    int i;
    int j;
};

class WayToGo
{
public:
    SimpleCoord coordsInverted[SIZE_MAX_WAY];
    SimpleCoord coordsShorted[SIZE_MAX_WAY];
    int current_idx = 0;
    int currentShorted_idx = 0;
    int length = 0;
    int lengthShorted = 0;
    void moveToNext()
    {
        currentShorted_idx++;
    }
    SimpleCoord getNext()
    {
        return coordsShorted[currentShorted_idx];
    }
    void simplify()
    {
        int count_short = 0;
        coordsShorted[0] = coordsInverted[length - 1];
        for (int i = 1; i < length - 1; i++)
        {
            // if (!(((coordsInverted[length - 2 - i].i == coordsInverted[length - 1 - i].i) && (coordsInverted[length - 1 - i].i == coordsInverted[length - i].i)) || ((coordsInverted[length - 2 - i].j == coordsInverted[length - 1 - i].j) && (coordsInverted[length - 1 - i].j == coordsInverted[length - i].j))))
            // {
            count_short++;
            coordsShorted[count_short] = coordsInverted[length - 1 - i];
            // }
        }
        count_short++;
        coordsShorted[count_short] = coordsInverted[0];
        lengthShorted = count_short + 1;
        currentShorted_idx = 0;
    }
    void reset()
    {
        current_idx = 0;
        currentShorted_idx = 0;
        length = 0;
        lengthShorted = 0;
    }
    int getLengthInverted()
    {
        return length;
    }
    int getLengthSorted()
    {
        return lengthShorted;
    }
    void pushArr(SimpleCoord *arr, int lenArr)
    {
        for (int i = 0; i < lenArr; i++)
        {
            coordsInverted[i] = arr[i];
        }
        length = lenArr;
    }

    bool hasFinish()
    {
        return (lengthShorted == 0) || (currentShorted_idx >= lengthShorted);
    }
};

struct mazeNode
{
    // mazeNode *parent = nullptr;
    int parent = 1;
    int cost = 0;
    int stopCriteria = 0;
    int id = -1;
    bool haveParent = false;
    int cheminsNb = 0;
    MazeSquare *square;
};

class listMazeNode
{
public:
    mazeNode elements[MAZE_NUMBER_CELLS];
    int last_idx = 0;
    void push(mazeNode node)
    {
        elements[last_idx] = node;
        last_idx++;
    }
    mazeNode pop()
    {
        last_idx = last_idx - 1;
        return elements[last_idx];
    }
    int len()
    {
        return last_idx;
    }
    mazeNode *get(int idx)
    {
        return &elements[idx];
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
    mazeNode elements[MAZE_NUMBER_CELLS];
    void reset()
    {
        for (int i = 0; i < MAZE_NUMBER_CELLS; i++)
        {
            elements[i].id = -1;
            elements[i].stopCriteria = 0;
        }
    }
    void add(mazeNode node)
    {
        elements[node.id].id = node.id;
        elements[node.id].square = node.square;
        elements[node.id].parent = node.parent;
        elements[node.id].cost = node.cost;
    }
    void add2(int id, mazeNode parent, int cost, MazeSquare *square, int stopCriteria_, int newcheminsNb_)
    {
        elements[id].id = id;
        elements[id].square = square;
        elements[id].parent = parent.id;
        elements[id].cost = cost;
        elements[id].stopCriteria = stopCriteria_;
        elements[id].cheminsNb = newcheminsNb_;
    }
    mazeNode *get(int id)
    {
        return &elements[id];
    }
    bool has(mazeNode node)
    {
        return (elements[node.id].id != -1);
    }
};

float distance(SimpleCoord A, SimpleCoord B);
bool isBoundarie(mazeNode node, int deleted = 0);
int genId(int i, int j);
int geti(int id);
int getj(int id);
int genId(const MazeSquare *start_);
bool isBoundarie(int i, int j, int deleted);
bool isBoundarie(mazeNode node, int deleted);
void getNeighborS(mazeNode *workingNode_, Gladiator *glad);
mazeNode extractMinCost(listMazeNode *frontier, Gladiator *glad);
int cost(mazeNode nodeA, mazeNode nodeB, int deleted, Gladiator *glad, States state);
hashMazeNode *solve(const MazeSquare *start_, Gladiator *glad, int pathLength, int deleted, States state = DEFAULT_STATE);
void printPath(hashMazeNode *costs, mazeNode A, mazeNode B, Gladiator *glad);
int genPath(SimpleCoord *pointMission, hashMazeNode *costs, mazeNode A, mazeNode B, Gladiator *glad);