// conflictoracle.cpp
#include "conflictoracle.hpp"

int solve_mvc(const std::vector<std::pair<int, int>>& edges, 
                int vertex_count, 
                int optimal_size, 
                double cutoff_time, 
                int random_seed);    

ConflictOracle::ConflictOracle(std::vector<Point>& _starts, std::vector<Point>& _goals)
    : N(_starts.size()), starts(_starts), goals(_goals),
    h_edge_list(N), c_edge_list(N),
    updategraph_time_us(0), cal_mvc_time_us(0),
    poco_call_count(0),poco_result(0){

    // Initialize with full conflict graph
    // auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            ConflictRelation rel = getConflictRelation(starts[i], goals[i], starts[j], goals[j]);
            add_edge(i, j, rel);
        }
    }
    h_edges.reserve(N);
    matched_vertices.resize(N);
    degrees.resize(N);
}

void ConflictOracle::updategraph(const std::vector<Point>& new_starts) {
    // Find changed agents
    std::vector<int> changed_agents;
    std::vector<bool> is_changed(N, false);
    changed_agents.reserve(N);
    for (int i = 0; i < N; ++i) {
        if (starts[i] != new_starts[i]) {
            changed_agents.push_back(i);
            is_changed[i] = true;
        }
    }
    // Update current positions
    starts = new_starts;

    if (changed_agents.empty()) return; // No changes

    // Fast removal: only iterate through neighbors of changed agents
    for (int i : changed_agents) {
        // Remove all edges involving agent i
        for (uint16_t j : h_edge_list[i]) {
            fast_remove(h_edge_list[j], i);
        }
        for (uint16_t j : c_edge_list[i]) {
            fast_remove(c_edge_list[j], i);
        }
        h_edge_list[i].clear();
        c_edge_list[i].clear();
    }

    // Add new conflicts for changed agents
    for (int i : changed_agents) {
        for (int j = 0; j < i; ++j) {
            if (is_changed[j]) continue; // avoid duplicate computation
            ConflictRelation rel = getConflictRelation(starts[i], goals[i], starts[j], goals[j]);
            add_edge(i, j, rel);
        }
        for (int j = i+1; j < N; ++j) {
            ConflictRelation rel = getConflictRelation(starts[i], goals[i], starts[j], goals[j]);
            add_edge(i, j, rel);
        }
    }
}

int ConflictOracle::solve_mvc_lb(){
    if (N <= 1) return 0;

    // 1-one lower bound of vertex cover based on maximal match
    int Match = 0, total_edge = 0;
    std::fill(matched_vertices.begin(), matched_vertices.end(), false);
    std::fill(degrees.begin(), degrees.end(), 0);

    for (uint16_t i = 0; i < N; ++i) {
        for (uint16_t j : h_edge_list[i]) {
            if (i >= j) continue;
            total_edge++;
            degrees[i]++;
            degrees[j]++;
            if (!matched_vertices[i] && !matched_vertices[j]) {
                Match++;
                matched_vertices[i] = true;
                matched_vertices[j] = true;
            }
        }
    }

    // 2-another lower bound of vertex cover based on degree sequence
    std::sort(degrees.begin(), degrees.end(), std::greater<size_t>());// soreted in decreasing order
    size_t c = 0, i = 0;
    for (; i < N; ++i) {
        c += degrees[i];
        if(c >= total_edge) break;
    }
    // return two lower bounds of minimum vertex cover
    return std::max(Match, int(i)+1);
}

bool ConflictOracle::update_calmvc(const std::vector<Point>& new_starts, bool cal_lb_mvc, int& lb, int& hedges, int& cedges) {
    // 1-update the graph structure first
    poco_call_count++;
    auto start = std::chrono::high_resolution_clock::now();

    updategraph(new_starts);
    hedges = get_hostile_count(); cedges = get_compatible_count();

    auto end1 = std::chrono::high_resolution_clock::now();
    updategraph_time_us += std::chrono::duration_cast<std::chrono::microseconds>(end1 - start).count();

    // 2- cal mvc
    if(cal_lb_mvc || mvc_solver_ == nullptr){
        // Use a mvc lb function solve_mvc_lb
        lb = solve_mvc_lb();
    }else{
        // Use NumvcSolver instead of solve_mvc_lb
        h_edges.clear();
        for (uint16_t i = 0; i < N; ++i) {
            for (uint16_t j : h_edge_list[i]) {
                if (i < j)  h_edges.push_back(std::pair(i,j));
            }
        }
        lb = mvc_solver_(h_edges, N);
    }
    auto end2 = std::chrono::high_resolution_clock::now();
    cal_mvc_time_us += std::chrono::duration_cast<std::chrono::microseconds>(end2 - end1).count();
    poco_result += lb;
    return true;
}
