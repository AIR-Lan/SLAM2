
#pragma once

//STL
#include <utility>

namespace ORB_SLAM3 {

class LineIterator {
public:

    LineIterator(const double x1_, const double y1_, const double x2_, const double y2_);

    bool getNext(std::pair<int, int> &pixel);

private:

    const bool steep;
    double x1, y1, x2, y2;

    double dx, dy, error;

    int maxX, ystep;
    int y, x;
};

} // namespace ORB_SLAM3
