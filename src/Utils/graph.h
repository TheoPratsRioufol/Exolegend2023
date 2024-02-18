
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
    DEFENSE,
    CRITICAL_RECOVERY,
    CRITICAL_RECOVERY_WAIT
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
    SimpleCoord trucAAjouter;
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
    bool isCoherent(Gladiator *glad, SimpleCoord c1, SimpleCoord c2)
    {
        if (c1.i == c2.i)
            return true;
        if (c1.j == c2.j)
            return true;
        if (c1.i - c2.i == 1 && c1.j - c2.j == 1)
        {
            if (glad->maze->getSquare(c1.i, c1.j)->southSquare == nullptr)
            {
                trucAAjouter.i = c1.i - 1;
                trucAAjouter.j = c1.j;
            }
            else
            {
                trucAAjouter.i = c1.i;
                trucAAjouter.j = c1.j - 1;
            }
        }
        if (c1.i - c2.i == -1 && c1.j - c2.j == 1)
        {
            if (glad->maze->getSquare(c1.i, c1.j)->southSquare == nullptr)
            {
                trucAAjouter.i = c1.i + 1;
                trucAAjouter.j = c1.j;
            }
            else
            {
                trucAAjouter.i = c1.i;
                trucAAjouter.j = c1.j - 1;
            }
        }
        if (c1.i - c2.i == 1 && c1.j - c2.j == -1)
        {
            if (glad->maze->getSquare(c1.i, c1.j)->northSquare == nullptr)
            {
                trucAAjouter.i = c1.i - 1;
                trucAAjouter.j = c1.j;
            }
            else
            {
                trucAAjouter.i = c1.i;
                trucAAjouter.j = c1.j + 1;
            }
        }
        if (c1.i - c2.i == -1 && c1.j - c2.j == -1)
        {
            if (glad->maze->getSquare(c1.i, c1.j)->northSquare == nullptr)
            {
                trucAAjouter.i = c1.i + 1;
                trucAAjouter.j = c1.j;
            }
            else
            {
                trucAAjouter.i = c1.i;
                trucAAjouter.j = c1.j + 1;
            }
        }
        return false;
    }
    void simplify(Gladiator *glad)
    {
        int count_short = 0;
        coordsShorted[0] = coordsInverted[length - 1];
        for (int i = 1; i < length - 1; i++)
        {
            if (!(((coordsInverted[length - 2 - i].i == coordsInverted[length - 1 - i].i) && (coordsInverted[length - 1 - i].i == coordsInverted[length - i].i)) || ((coordsInverted[length - 2 - i].j == coordsInverted[length - 1 - i].j) && (coordsInverted[length - 1 - i].j == coordsInverted[length - i].j))))
            {
                if (!isCoherent(glad, coordsInverted[length - 2 - i], coordsInverted[length - 1 - i]))
                {
                    glad->log("XXXXXXXXXXXDDDDDDDDD");
                    count_short++;
                    coordsShorted[count_short] = trucAAjouter;
                }
                count_short++;
                coordsShorted[count_short] = coordsInverted[length - 1 - i];
            }
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
    void pushSingleCoord(SimpleCoord c1, SimpleCoord c2)
    {
        coordsShorted[0] = c1;
        coordsShorted[1] = c2;
        currentShorted_idx = 0;
        length = 2;
    }
    void goToMaze(Gladiator *glad, int deleted)
    {

        int xtarget = glad->maze->getNearestSquare()->i;
        int ytarget = glad->maze->getNearestSquare()->j;

        if (xtarget < deleted + 1)
            xtarget = deleted + 1;
        if (xtarget > NB_ROW - deleted - 2)
            xtarget = NB_ROW - deleted - 2;

        if (ytarget < deleted + 1)
            ytarget = deleted + 1;
        if (ytarget > NB_ROW - deleted - 2)
            ytarget = NB_ROW - deleted - 2;

        pushSingleCoord(SimpleCoord{glad->maze->getNearestSquare()->i, glad->maze->getNearestSquare()->j}, SimpleCoord{xtarget, ytarget});
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
    float cost = 0;
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
        elements[last_idx].id = node.id;
        elements[last_idx].square = node.square;
        elements[last_idx].parent = node.parent;
        elements[last_idx].cost = node.cost;
        elements[last_idx].stopCriteria = node.stopCriteria;
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
    bool has(int id)
    {
        return (elements[id].id != -1);
    }
};

const SimpleCoord CENTER_POINT = SimpleCoord{5, 5};

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
float cost(mazeNode nodeA, mazeNode nodeB, int deleted, Gladiator *glad, States state);
hashMazeNode *solve(const MazeSquare *start_, Gladiator *glad, int pathLength, int deleted, States state = DEFAULT_STATE);
void printPath(hashMazeNode *costs, mazeNode A, mazeNode B, Gladiator *glad);
int genPath(SimpleCoord *pointMission, hashMazeNode *costs, mazeNode A, mazeNode B, Gladiator *glad);