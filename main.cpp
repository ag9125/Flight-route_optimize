#include <iostream>
#include <fstream>
#include "Graph.h"

void runAllAstar(Graph& g, const std::string& src, const std::string& dst) {
    unsigned int distCost = 0, flightHops = 0, minCost = 0;

    auto path1 = g.AstarDistance(src, dst, distCost);
    auto path2 = g.AstarFlights(src, dst, flightHops);
    auto path3 = g.AstarCost(src, dst, minCost);

    auto print = [](const std::string& title, const std::vector<std::string>& path, unsigned int value, const std::string& unit) {
        std::cout << "\n--- " << title << " ---\n";
        if (path.empty()) std::cout << "No path found.\n";
        else {
            std::cout << "Path: ";
            for (size_t i = 0; i < path.size(); ++i)
                std::cout << path[i] << (i + 1 < path.size() ? " -> " : "\n");
            std::cout << title << ": " << value << " " << unit << "\n";
        }
    };

    print("Shortest Distance", path1, distCost, "meters");
    print("Minimum Flights", path2, flightHops, "flights");
    print("Cheapest Path", path3, minCost, "rupees");
}

int main() {
    Graph g;
    std::ifstream fin("input.txt");
    if (!fin) {
        std::cerr << "Could not open input.txt\n";
        return 1;
    }

    int n;
    fin >> n;
    for (int i = 0; i < n; ++i) {
        std::string name;
        double lat, lon;
        fin >> name >> lat >> lon;
        g.addNode(name, lat, lon);
    }

    int m;
    fin >> m;
    for (int i = 0; i < m; ++i) {
        std::string from, to;
        unsigned int distance, cost;
        fin >> from >> to >> distance >> cost;
        g.addEdge(from, to, distance, cost);
    }

    std::string source, dest;
    fin >> source >> dest;
    runAllAstar(g, source, dest);

    return 0;
}
