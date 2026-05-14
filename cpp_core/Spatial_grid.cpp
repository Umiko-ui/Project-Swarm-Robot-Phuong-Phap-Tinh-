#include "Spatial_grid.h"
#include <cmath>

SpatialGrid::SpatialGrid(double cellSize) : cellSize(cellSize) {}

void SpatialGrid::clear() {
    grid.clear();
}

long long SpatialGrid::hash(int cx, int cy) const {
    return ((long long)cx << 32) ^ (unsigned int)cy;
}

void SpatialGrid::insert(int i, std::vector<Robot>& robots) {
    int cx = std::floor(robots[i].x / cellSize);
    int cy = std::floor(robots[i].y / cellSize);

    robots[i].cx = cx;
    robots[i].cy = cy;

    grid[hash(cx, cy)].push_back(i);
}