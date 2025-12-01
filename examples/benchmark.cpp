#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include "conflictoracle.hpp"

void benchmark_conflict_detection(int num_pairs) {
    std::cout << "Benchmarking pairwise conflict detection..." << std::endl;
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 50);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    int hostile_count = 0, compatible_count = 0, free_count = 0;
    
    for (int i = 0; i < num_pairs; ++i) {
        Point s1(dis(gen), dis(gen)), g1(dis(gen), dis(gen));
        Point s2(dis(gen), dis(gen)), g2(dis(gen), dis(gen));
        
        ConflictRelation rel = getConflictRelation(s1, g1, s2, g2);
        
        switch(rel) {
            case ConflictRelation::Hostile: hostile_count++; break;
            case ConflictRelation::Compatible: compatible_count++; break;
            case ConflictRelation::Free: free_count++; break;
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    std::cout << "Results for " << num_pairs << " agent pairs:" << std::endl;
    std::cout << "  - Hostile: " << hostile_count << " (" << (100.0 * hostile_count / num_pairs) << "%)" << std::endl;
    std::cout << "  - Compatible: " << compatible_count << " (" << (100.0 * compatible_count / num_pairs) << "%)" << std::endl;
    std::cout << "  - Free: " << free_count << " (" << (100.0 * free_count / num_pairs) << "%)" << std::endl;
    std::cout << "  - Total time: " << duration.count() << " μs" << std::endl;
    std::cout << "  - Average per pair: " << (duration.count() / (double)num_pairs) << " μs" << std::endl;
}

void benchmark_oracle_scaling(const std::vector<int>& agent_counts) {
    std::cout << "\nBenchmarking ConflictOracle scaling..." << std::endl;
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 20);
    
    for (int N : agent_counts) {
        std::vector<Point> starts, goals;
        for (int i = 0; i < N; ++i) {
            starts.push_back(Point(dis(gen), dis(gen)));
            goals.push_back(Point(dis(gen), dis(gen)));
        }
        
        ConflictOracle oracle(starts, goals);
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Perform 10 updates
        for (int iter = 0; iter < 10; ++iter) {
            std::vector<Point> new_starts;
            for (int i = 0; i < N; ++i) {
                new_starts.push_back(Point(dis(gen), dis(gen)));
            }
            
            int mvc, hedges, cedges;
            oracle.update_calmvc(new_starts, true, mvc, hedges, cedges);
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        std::cout << "N=" << N << " agents:" << std::endl;
        std::cout << "  - 10 updates: " << duration.count() << " ms" << std::endl;
        std::cout << "  - Avg per update: " << (duration.count() / 10.0) << " ms" << std::endl;
        std::cout << "  - Graph update time: " << oracle.get_updategraph_time() << " μs" << std::endl;
        std::cout << "  - MVC time: " << oracle.get_cal_mvc_time() << " μs" << std::endl;
    }
}

int main() {
    std::cout << "=== POCO Performance Benchmark ===" << std::endl;
    
    // Benchmark pairwise conflict detection
    benchmark_conflict_detection(10000);
    
    // Benchmark oracle scaling
    std::vector<int> agent_counts = {10, 50, 100, 200, 500};
    benchmark_oracle_scaling(agent_counts);
    
    return 0;
}
