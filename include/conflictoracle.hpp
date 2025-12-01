#pragma once

#include "conflict.hpp"
#include <vector>
#include <chrono>
#include <unordered_set>

// a function 
using MVCSolverCallback = std::function<int(const std::vector<std::pair<int, int>>&, int)>;

class ConflictOracle {
private:
    const uint16_t N;  // number of agents
    std::vector<Point> starts;
    std::vector<Point> goals;

    // Adjacency lists for different conflict relations
    std::vector<std::vector<uint16_t>> h_edge_list;  // hostile edges adjacency list
    std::vector<std::vector<uint16_t>> c_edge_list;  // compatible edges adjacency list

    // Performance statistics
    long long updategraph_time_us;                   // Cumulative update graph time (microseconds)
    long long cal_mvc_time_us;                       // Cumulative MVC computation time (microseconds)
    uint poco_call_count;                            // update_calmvc call count
    uint poco_result;                                // update_calmvc results

    std::vector<std::pair<int, int>> h_edges;// edge list
    std::vector<bool> matched_vertices;      // data structure used in solve_mvc_lb
    std::vector<size_t> degrees;             // data structure used in solve_mvc_lb

    MVCSolverCallback mvc_solver_ = nullptr;
public:
    ConflictOracle(std::vector<Point>& _starts, std::vector<Point>& _goals);
    
    void set_mvc_solver(MVCSolverCallback solver) {
        mvc_solver_ = solver;
    }
    
    // Update current positions and return vertex cover lower bound. cal_lb_mvc = True: cal mvc lb, False: cal mvc by algos
    bool update_calmvc(const std::vector<Point>& new_starts, bool cal_lb_mvc, int &mvc, int& hedges, int& cedges);

    // cal lower bound of mvc using maximal matching
    int solve_mvc_lb();

    // update the graph structure based on current_positions
    void updategraph(const std::vector<Point>& new_starts);

    // Performance statistics getters
    long long get_updategraph_time() const { return updategraph_time_us; }
    long long get_cal_mvc_time() const { return cal_mvc_time_us; }
    uint get_poco_call_count() const { return poco_call_count; }
    uint get_poco_result() const { return poco_result; }
    
    // Helper functions
    void add_edge(uint16_t i, uint16_t j, ConflictRelation &rel) {
        switch(rel) {
            case ConflictRelation::Hostile:
                h_edge_list[i].push_back(j);
                h_edge_list[j].push_back(i);
                break;
            case ConflictRelation::Compatible:
                c_edge_list[i].push_back(j);
                c_edge_list[j].push_back(i);
                break;
            case ConflictRelation::Free:
                // Not stored
                break;
        }
    }
    
    // Search for 'val' in an unordered vector and remove it. Complexity O(N)
    void fast_remove(std::vector<uint16_t>& vec, uint16_t val) {
        for (size_t k = 0; k < vec.size(); ++k) {
            if (vec[k] == val) {
                vec[k] = vec.back(); // Overwrite the current element with the last element
                vec.pop_back();      // Remove the last element (which is now duplicate)
                return;              
            }
        }
    }

    void remove_edge(uint16_t i, uint16_t j) {
        fast_remove(h_edge_list[i],j);
        fast_remove(h_edge_list[j],i);
        fast_remove(c_edge_list[i],j);
        fast_remove(c_edge_list[j],i);
    }

    ConflictRelation get_agent_relation(uint16_t i, uint16_t j) const {
        auto& h_neis = h_edge_list[i];
        auto& c_neis = c_edge_list[i];
        if(std::find(h_neis.begin(),h_neis.end(),j) != h_neis.end()) return ConflictRelation::Hostile;
        if(std::find(c_neis.begin(),c_neis.end(),j) != c_neis.end()) return ConflictRelation::Compatible;
        return ConflictRelation::Free;
    }

    // Get current hostile/compatible edge counts
    size_t get_hostile_count() const {
        size_t count = 0;
        for (int i = 0; i < N; ++i) {
            count += h_edge_list[i].size();
        }
        return count / 2;  // each edge counted twice
    }

    size_t get_compatible_count() const {
        size_t count = 0;
        for (int i = 0; i < N; ++i) {
            count += c_edge_list[i].size();
        }
        return count / 2;  // each edge counted twice
    }
};
