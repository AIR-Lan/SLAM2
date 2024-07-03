

#pragma once

//STL
#include <list>
#include <unordered_set>
#include <utility>
#include <vector>

namespace ORB_SLAM3 {

void getLineCoords(double x1, double y1, double x2, double y2, std::list<std::pair<int, int>> &line_coords);

struct GridWindow {
    std::pair<int, int> width, height;
};

class GridStructure {
public:

    int rows, cols;

    GridStructure();
    
    GridStructure(int rows, int cols);

    ~GridStructure();

    std::list<int>& at(int x, int y);

    void get(int x, int y, const GridWindow &w, std::unordered_set<int> &indices) const;

    void clear();

private:

    std::vector<std::vector<std::list<int>>> grid;
    std::list<int> out_of_bounds;
};

} // namespace ORB_SLAM3
