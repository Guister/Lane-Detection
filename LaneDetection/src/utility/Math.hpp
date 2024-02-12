#pragma once

// C++ includes
#include <open3d/Open3D.h>
#include <fstream>
#include <unordered_map>
#include <string>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/QR>

// Project include
#include "../dataStructures/Point.hpp"

// Utility math functions 
namespace Math
{   
    // Helper functions to visualize intensity values
    std::unordered_map<std::string, int> buildIntensityMap(const std::vector<Point> &points);
    double returnAverageIntensity(const std::vector<Point> &points);

    // Helper functions to apply rotation
    double toRadians(int angle);
    Eigen::Matrix3d buildRotationMatrix(int angle);

    // Function that returns a polynomial fitting of given order
    std::vector<double> polyfit(const std::vector<double> &t, const std::vector<double> &v, int order);
}