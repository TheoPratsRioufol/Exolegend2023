// #include <iostream>
// #include <vector>
// #include <queue>
// #include <cmath>
// #include <stack>
// #include <algorithm>

// #include "planner.h"

// using namespace std;

// Node::Node(int x, int y, int g, int h, Node* parent)
//     : x(x), y(y), g(g), h(h), f(g + h), parent(parent) {}

// bool CompareNodes::operator()(const Node* a, const Node* b) {
//     return a->f > b->f;
// }

// int heuristic(int x1, int y1, int x2, int y2) {
//     return abs(x1 - x2) + abs(y1 - y2);
// }

// vector<pair<int, int>> findPath(vector<vector<int>>& maze, pair<int, int> start, pair<int, int> goal) {
//     int rows = maze.size(), cols = maze[0].size();
//     priority_queue<Node*, vector<Node*>, CompareNodes> openList;    //openlist
//     vector<vector<bool>> closedList(rows, vector<bool>(cols, false));  //closed list
//     vector<vector<Node*>> allNodes(rows, vector<Node*>(cols, nullptr));  //vector to avoid having multiple nodes created for 1 cell
    
//     Node* startNode = new Node(start.first, start.second, 0, heuristic(start.first, start.second, goal.first, goal.second));
//     openList.push(startNode);
//     allNodes[start.first][start.second] = startNode;
    
//     int dx[] = {-1, 1, 0, 0};     //directions around the current node
//     int dy[] = {0, 0, -1, 1};
    
//     while (!openList.empty()) {
//         Node* current = openList.top();
//         openList.pop();
        
//         if (current->x == goal.first && current->y == goal.second) {  //if we reached the goal
//             vector<pair<int, int>> path;     //initialization of the path vector
//             while (current) {
//                 path.push_back({current->x, current->y});
//                 current = current->parent;
//             }
//             reverse(path.begin(), path.end());    //this will need to be taken care of
//             return path;
//         }
        
//         closedList[current->x][current->y] = true;
        
//         for (int i = 0; i < 4; ++i) {
//             int nx = current->x + dx[i], ny = current->y + dy[i];
            
//             if (nx >= 0 && nx < rows && ny >= 0 && ny < cols && maze[nx][ny] == 0 && !closedList[nx][ny]) {
//                 int newG = current->g + 1;
//                 if (!allNodes[nx][ny] || newG < allNodes[nx][ny]->g) {
//                     Node* neighbor = new Node(nx, ny, newG, heuristic(nx, ny, goal.first, goal.second), current);
//                     openList.push(neighbor);
//                     allNodes[nx][ny] = neighbor;
//                 }
//             }
//         }
//     }
//     return {};
// }

// void printMaze(vector<vector<int>>& maze, const vector<pair<int, int>>& path) {
//     for (auto [x, y] : path) {
//         if (maze[x][y] == 0)
//             maze[x][y] = 2;
//     }
//     for (int i = 0; i < maze.size(); i++) {
//         for (int j = 0; j < maze[0].size(); j++) {
//             if (maze[i][j] == 1)
//                 cout << "# ";
//             else if (maze[i][j] == 2)
//                 cout << ". ";
//             else
//                 cout << "  ";
//         }
//         cout << endl;
//     }
// }

// // int main() {
// //     vector<vector<int>> maze = {
// //         {0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
// //         {1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
// //         {0, 0, 0, 0, 1, 0, 0, 0, 1, 0},
// //         {0, 1, 1, 1, 1, 1, 1, 0, 1, 0},
// //         {0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
// //         {1, 1, 1, 1, 1, 0, 1, 1, 1, 0},
// //         {0, 1, 0, 0, 0, 0, 0, 0, 1, 0},
// //         {0, 1, 0, 1, 1, 1, 1, 0, 1, 0},
// //         {0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
// //         {1, 1, 1, 1, 1, 1, 1, 0, 0, 0}
// //     };
    
// //     pair<int, int> start = {0, 0};
// //     pair<int, int> goal = {9, 9};
    
// //     vector<pair<int, int>> path = findPath(maze, start, goal);
    
// //     if (!path.empty()) {
// //         cout << "Path found:\n";
// //         printMaze(maze, path);
// //     } else {
// //         cout << "No path found." << endl;
// //     }
    
// //     return 0;
// // }
