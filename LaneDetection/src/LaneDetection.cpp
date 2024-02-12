// C++ includes
#include <iostream>
#include <iterator>
#include <algorithm>
#include <open3d/Open3D.h>
#include <Eigen/Dense>

// Project includes
#include "dataStructures/Point.hpp"
#include "dataStructures/Pointcloud.hpp"
#include "utility/Math.hpp"

int main()
{
    open3d::PrintOpen3DVersion();

    std::ifstream file("/home/guister/Desktop/LaneDetection/data/pointclouds/1553567105504169477.bin", std::ios::binary);    
    Pointcloud pointcloud(file);

    auto intensityValues = Math::buildIntensityMap(pointcloud.getPoints());

    pointcloud.applyFilters(10, 0, {30,4}, {10, -5, 3, -2}, {0.1,3}, 0.05, true);
    
    auto answer = pointcloud.calculatePolynomial({-100, 2, 100}, 0.8);
}
