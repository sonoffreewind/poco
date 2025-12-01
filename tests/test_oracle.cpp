#include <iostream>
#include <vector>
#include <chrono>
#include <cassert>
#include "conflictoracle.hpp"

void test_oracle_basic() {
    std::cout << "Testing ConflictOracle basic functionality..." << std::endl;
    
    // Create 4 agents
    std::vector<Point> starts = {Point(0,0), Point(2,0), Point(0,2), Point(2,2)};
    std::vector<Point> goals = {Point(1,1), Point(1,0), Point(1,2), Point(1,1)};
    
    ConflictOracle oracle(starts, goals);
    
    // Test initial state
    assert(oracle.get_poco_call_count() == 0);
    
    // Update with new positions
    std::vector<Point> new_starts = {Point(0,1), Point(2,1), Point(1,2), Point(2,1)};
    int mvc, hedges, cedges;
    
    bool result = oracle.update_calmvc(new_starts, true, mvc, hedges, cedges);
    assert(result == true);
    assert(oracle.get_poco_call_count() == 1);
    
    std::cout << "✓ Oracle basic tests passed" << std::endl;
    std::cout << "  - MVC: " << mvc << ", Hostile edges: " << hedges << ", Compatible edges: " << cedges << std::endl;
}

void test_oracle_performance() {
    std::cout << "Testing ConflictOracle performance..." << std::endl;
    
    const int N = 100;  // 100 agents
    std::vector<Point> starts, goals;
    
    // Generate random-like positions
    for (int i = 0; i < N; ++i) {
        starts.push_back(Point(i % 10, i / 10));
        goals.push_back(Point((i + 5) % 10, (i + 50) / 10));
    }
    
    ConflictOracle oracle(starts, goals);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Perform multiple updates
    for (int iter = 0; iter < 10; ++iter) {
        std::vector<Point> new_starts;
        for (int i = 0; i < N; ++i) {
            new_starts.push_back(Point((i + iter) % 10, (i + iter * 10) / 10));
        }
        
        int mvc, hedges, cedges;
        oracle.update_calmvc(new_starts, true, mvc, hedges, cedges);
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "✓ Performance test completed" << std::endl;
    std::cout << "  - " << N << " agents, 10 iterations: " << duration.count() << "ms" << std::endl;
    std::cout << "  - Total update graph time: " << oracle.get_updategraph_time() << "μs" << std::endl;
    std::cout << "  - Total MVC time: " << oracle.get_cal_mvc_time() << "μs" << std::endl;
}

int main() {
    std::cout << "=== POCO Oracle Tests ===" << std::endl;
    
    try {
        test_oracle_basic();
        test_oracle_performance();
        std::cout << "\n=== All oracle tests passed! ===" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
