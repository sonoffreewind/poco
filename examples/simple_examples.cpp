#include <iostream>
#include <vector>
#include "conflict.hpp"
#include "conflictoracle.hpp"

int main() {
    std::cout << "=== POCO Simple Example ===" << std::endl;
    
    // Example 1: Basic conflict detection
    std::cout << "\n1. Basic Conflict Detection:" << std::endl;
    
    Point agent1_start(0, 0), agent1_goal(2, 0);
    Point agent2_start(1, 0), agent2_goal(3, 0);
    
    ConflictRelation relation = getConflictRelation(agent1_start, agent1_goal, agent2_start, agent2_goal);
    
    std::cout << "Agent 1: (" << agent1_start.x << "," << agent1_start.y << ") -> (" 
              << agent1_goal.x << "," << agent1_goal.y << ")" << std::endl;
    std::cout << "Agent 2: (" << agent2_start.x << "," << agent2_start.y << ") -> (" 
              << agent2_goal.x << "," << agent2_goal.y << ")" << std::endl;
    
    std::string relation_str;
    switch(relation) {
        case ConflictRelation::Free: relation_str = "Free"; break;
        case ConflictRelation::Compatible: relation_str = "Compatible"; break;
        case ConflictRelation::Hostile: relation_str = "Hostile"; break;
    }
    std::cout << "Conflict Relation: " << relation_str << std::endl;
    
    // Example 2: Multi-agent conflict oracle
    std::cout << "\n2. Multi-Agent Conflict Oracle:" << std::endl;
    
    std::vector<Point> starts = {
        Point(0, 0), Point(3, 0), Point(0, 3), Point(3, 3)
    };
    std::vector<Point> goals = {
        Point(3, 3), Point(0, 3), Point(3, 0), Point(0, 0)
    };
    
    ConflictOracle oracle(starts, goals);
    
    // Simulate agent movement
    std::vector<Point> current_positions = {
        Point(1, 1), Point(2, 1), Point(1, 2), Point(2, 2)
    };
    
    int mvc_size, hostile_edges, compatible_edges;
    oracle.update_calmvc(current_positions, true, mvc_size, hostile_edges, compatible_edges);
    
    std::cout << "Current positions updated:" << std::endl;
    for (size_t i = 0; i < current_positions.size(); ++i) {
        std::cout << "  Agent " << i << ": (" << current_positions[i].x 
                  << "," << current_positions[i].y << ")" << std::endl;
    }
    
    std::cout << "Conflict Analysis:" << std::endl;
    std::cout << "  - Minimum Vertex Cover Size: " << mvc_size << std::endl;
    std::cout << "  - Hostile Edges: " << hostile_edges << std::endl;
    std::cout << "  - Compatible Edges: " << compatible_edges << std::endl;
    
    // Example 3: Performance statistics
    std::cout << "\n3. Performance Statistics:" << std::endl;
    std::cout << "  - Update calls: " << oracle.get_poco_call_count() << std::endl;
    std::cout << "  - Graph update time: " << oracle.get_updategraph_time() << " μs" << std::endl;
    std::cout << "  - MVC computation time: " << oracle.get_cal_mvc_time() << " μs" << std::endl;
    
    return 0;
}
