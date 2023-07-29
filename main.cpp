#include <iostream>
#include <vector>
#include <unordered_set>
#include <cmath>
#include <queue>
#include <ros/ros.h>

struct Node {
    int x, y, z;
    bool isObstacle;
    float cost;
    Node* parent;

    Node(int x = 0, int y = 0, int z = 0, bool isObstacle = false, float cost = std::numeric_limits<float>::infinity(), Node* parent = nullptr)
        : x(x), y(y), z(z), isObstacle(isObstacle), cost(cost), parent(parent) {}

    void reset() {
        cost = std::numeric_limits<float>::infinity();
        parent = nullptr;
    }
};

class AStar {
public:
    AStar();
    ~AStar();

    std::vector<Node*> findPath(Node* start, Node* goal, std::vector<std::vector<std::vector<Node>>>& grid);
    std::vector<Node*> getNeighbors(Node* node, std::vector<std::vector<std::vector<Node>>>& grid);
    float heuristic(Node* a, Node* b);
};

std::vector<Node*> findPathAvoidingConflict(AStar& aStar, Node* start, Node* goal, Node* conflictNode, std::vector<std::vector<std::vector<Node>>>& grid) {
    conflictNode->isObstacle = true;
    std::vector<Node*> newPath = aStar.findPath(start, goal, grid);
    conflictNode->isObstacle = false;
    return newPath;
}

void resetGrid(std::vector<std::vector<std::vector<Node>>>& grid) {
    for (auto& matrix : grid) {
        for (auto& row : matrix) {
            for (auto& node : row) {
                node.reset();
            }
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "astar_simulation_node");
    ros::NodeHandle nh;

    int size = 100;
    std::vector<std::vector<std::vector<Node>>> grid(size, std::vector<std::vector<Node>>(size, std::vector<Node>(size)));

    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            for (int k = 0; k < size; k++) {
                grid[i][j][k] = Node(i, j, k);
            }
        }
    }

    int start1_x, start1_y, start1_z, goal1_x, goal1_y, goal1_z;
    int start2_x, start2_y, start2_z, goal2_x, goal2_y, goal2_z;

    std::cout << "Enter the starting point for agent 1 (x y z): ";
    std::cin >> start1_x >> start1_y >> start1_z;
    std::cout << "Enter the goal point for agent 1 (x y z): ";
    std::cin >> goal1_x >> goal1_y >> goal1_z;

    std::cout << "Enter the starting point for agent 2 (x y z): ";
    std::cin >> start2_x >> start2_y >> start2_z;
    std::cout << "Enter the goal point for agent 2 (x y z): ";
    std::cin >> goal2_x >> goal2_y >> goal2_z;

    Node* start1 = &grid[start1_x][start1_y][start1_z];
    Node* goal1 = &grid[goal1_x][goal1_y][goal1_z];
    Node* start2 = &grid[start2_x][start2_y][start2_z];
    Node* goal2 = &grid[goal2_x][goal2_y][goal2_z];

    AStar aStar;
    Agent agent1(start1, goal1);
    agent1.path = aStar.findPath(agent1.start, agent1.goal, grid);

    resetGrid(grid); // 생성한 경로를 reset하여 agent2의 경로를 저장하기 위해서
    Agent agent2(start2, goal2);
    agent2.path = aStar.findPath(agent2.start, agent2.goal, grid);

    std::unordered_set<Node*> pathSet(agent1.path.begin(), agent1.path.end());

    for (Node* node : agent2.path) {
        if (pathSet.count(node)) {
            std::vector<Node*> newPath = findPathAvoidingConflict(aStar, agent2.start, agent2.goal, node, grid);
            if (!newPath.empty() && agent2.path.size() > newPath.size()) {
                agent2.path = newPath;
            }
        }
    }

    std::cout << "Path for agent 1:\n";
    for (auto node : agent1.path) {
        std::cout << "(" << node->x << ", " << node->y << ", " << node->z << ")\n";
    }

    std::cout << "Path for agent 2:\n";
    for (auto node : agent2.path) {
        std::cout << "(" << node->x << ", " << node->y << ", " << node->z << ")\n";
    }

    ros::spin();

    return 0;
}
