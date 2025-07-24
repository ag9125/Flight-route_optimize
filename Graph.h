#pragma once
#include <vector>
#include <unordered_map>
#include <string>

class Graph {
private:
    int size = 0;
    std::unordered_map<std::string, int> ids;
    std::vector<std::string> names;
    std::vector<std::vector<std::pair<int, unsigned int>>> adj_list;
    std::vector<std::vector<std::pair<int, unsigned int>>> cost_list;
    std::vector<std::pair<double, double>> coordinates;

    double toRadians(double degree);
    unsigned int heuristicDistance(int u, int v);
    std::vector<std::string> reconstruct_path(int start, int to, std::vector<int>& came_from);

public:
    void addNode(const std::string& name, double lat, double lon);
    void addEdge(const std::string& from, const std::string& to, unsigned int distance, unsigned int cost);

    std::vector<std::string> AstarDistance(const std::string& from, const std::string& to, unsigned int& costOut);
    std::vector<std::string> AstarFlights(const std::string& from, const std::string& to, unsigned int& hopsOut);
    std::vector<std::string> AstarCost(const std::string& from, const std::string& to, unsigned int& costOut);
};
