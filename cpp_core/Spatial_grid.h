#pragma once
#include <vector>
#include <unordered_map>

struct Robot {
    double x, y;
    int cx, cy;
    double vx = 0.0;
    double vy = 0.0;
    
};

class SpatialGrid {
public:
    double cellSize;
    std::unordered_map<long long, std::vector<int>> grid;
    
    SpatialGrid(double cellSize);

    void clear();
    long long hash(int cx, int cy) const;
    void insert(int i, std::vector<Robot>& robots);
};