#include "Graph.h"
#include <queue>
#include <cmath>
#include <algorithm>
#include <limits>

double Graph::toRadians(double degree) {
    return degree * M_PI / 180.0;
}

unsigned int Graph::heuristicDistance(int u, int v) {
    auto [lat1, lon1] = coordinates[u];
    auto [lat2, lon2] = coordinates[v];
    lat1 = toRadians(lat1); lon1 = toRadians(lon1);
    lat2 = toRadians(lat2); lon2 = toRadians(lon2);

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    double a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
    double c = 2 * asin(sqrt(a));
    return static_cast<unsigned int>(c * 6371000); // meters
}

void Graph::addNode(const std::string& name, double lat, double lon) {
    if (ids.find(name) != ids.end()) return;
    ids[name] = size++;
    names.push_back(name);
    coordinates.emplace_back(lat, lon);
    adj_list.emplace_back();
    cost_list.emplace_back();
}

void Graph::addEdge(const std::string& from, const std::string& to, unsigned int distance, unsigned int cost) {
    if (ids.find(from) == ids.end() || ids.find(to) == ids.end()) return;
    int u = ids[from], v = ids[to];
    adj_list[u].emplace_back(v, distance);
    cost_list[u].emplace_back(v, cost);
}

std::vector<std::string> Graph::reconstruct_path(int start, int to, std::vector<int>& came_from) {
    std::vector<std::string> path;
    if (came_from[to] == -1) return path;
    for (int at = to; at != start; at = came_from[at])
        path.push_back(names[at]);
    path.push_back(names[start]);
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<std::string> Graph::AstarDistance(const std::string& from, const std::string& to, unsigned int& costOut) {
    int src = ids[from], dst = ids[to];
    std::vector<unsigned int> g(size, UINT_MAX);
    std::vector<int> came_from(size, -1);
    using P = std::pair<unsigned int, int>;
    std::priority_queue<P, std::vector<P>, std::greater<>> pq;

    g[src] = 0;
    pq.emplace(heuristicDistance(src, dst), src);

    while (!pq.empty()) {
        auto [f, u] = pq.top(); pq.pop();
        if (u == dst) break;

        for (auto& [v, w] : adj_list[u]) {
            unsigned int tentative = g[u] + w;
            if (tentative < g[v]) {
                g[v] = tentative;
                came_from[v] = u;
                pq.emplace(g[v] + heuristicDistance(v, dst), v);
            }
        }
    }

    costOut = g[dst];
    return (g[dst] == UINT_MAX) ? std::vector<std::string>{} : reconstruct_path(src, dst, came_from);
}

std::vector<std::string> Graph::AstarFlights(const std::string& from, const std::string& to, unsigned int& hopsOut) {
    int src = ids[from], dst = ids[to];
    std::vector<unsigned int> g(size, UINT_MAX);
    std::vector<int> came_from(size, -1);
    using P = std::pair<unsigned int, int>;
    std::priority_queue<P, std::vector<P>, std::greater<>> pq;

    g[src] = 0;
    pq.emplace(0, src); // No good heuristic for hops

    while (!pq.empty()) {
        auto [hops, u] = pq.top(); pq.pop();
        if (u == dst) break;

        for (auto& [v, _] : adj_list[u]) {
            unsigned int tentative = g[u] + 1;
            if (tentative < g[v]) {
                g[v] = tentative;
                came_from[v] = u;
                pq.emplace(tentative, v);
            }
        }
    }

    hopsOut = g[dst];
    return (g[dst] == UINT_MAX) ? std::vector<std::string>{} : reconstruct_path(src, dst, came_from);
}

std::vector<std::string> Graph::AstarCost(const std::string& from, const std::string& to, unsigned int& costOut) {
    int src = ids[from], dst = ids[to];
    std::vector<unsigned int> g(size, UINT_MAX);
    std::vector<int> came_from(size, -1);
    using P = std::pair<unsigned int, int>;
    std::priority_queue<P, std::vector<P>, std::greater<>> pq;

    g[src] = 0;
    pq.emplace(heuristicDistance(src, dst), src); // We reuse heuristicDistance for tie-breaking

    while (!pq.empty()) {
        auto [cost, u] = pq.top(); pq.pop();
        if (u == dst) break;

        for (auto& [v, c] : cost_list[u]) {
            unsigned int tentative = g[u] + c;
            if (tentative < g[v]) {
                g[v] = tentative;
                came_from[v] = u;
                pq.emplace(g[v] + heuristicDistance(v, dst), v);
            }
        }
    }

    costOut = g[dst];
    return (g[dst] == UINT_MAX) ? std::vector<std::string>{} : reconstruct_path(src, dst, came_from);
}
