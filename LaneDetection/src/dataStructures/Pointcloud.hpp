#pragma once

// C++ includes
#include <open3d/Open3D.h>
#include <memory>
#include <fstream>
#include <algorithm>

// Project includes
#include "Point.hpp"
#include "../utility/Math.hpp"
#include "../utility/DataType.hpp"

/*
* Pointcloud class wrapper.
* Contains Open3d's pointcloud, a vector of type Point with equivalent information and a rotation angle.
* Contains functions that transform and manipulate its data.
*/
class Pointcloud {
private:

    std::shared_ptr<open3d::geometry::PointCloud> pointcloud;
    std::vector<Point> points;
    float angle;

    //Data manipulation
    void parseData(std::ifstream &file);
    void populatePointcloud();
    void populatePointVector();
    void update(DataType dataType);

    // Filters
    // Remove all points with intensity below intensity value
    void intensityFilter(int intensity);
    // Rotate the coordinate system by angle value
    void rotationFilter(float angle);    
    // Remove all points with x and y coordinates below and above x and y values
    void distanceFilter(float x, float y);
    // Remove all points above top, below bot, and between innerTop and innerBot
    void sliceFilter(float top, float bot, float innerTop, float innerBot);
    // Remove all points accused by the DBSCAN algorithm
    void clusterDBSCAN(float density, int minPoints);
    // Remove all non finite and duplicated points
    void clean();
    // Remove points by dividing the pointcloud into voxels
    void voxelDownSample(double voxelSize);

    // Separates the bottom lane from the top lane.
    // Separator is a Point that represents then end of the bottom lane
    // and the start of the top lane.
    // Example conditions: x: value below any other point
    //                     y: value above bottom lane highest point + proximity
    //                     z: value unique
    // Proximity used for pointSort function.
    void separateLanes(const Point &separator, float proximity,
     std::pair<std::vector<double>, std::vector<double>> &botLane,
     std::pair<std::vector<double>, std::vector<double>> &topLane);

    // Sort points by x coordinate, bottom lane first, then top lane.
    // Proximity value represents how close a point's Y coordinate
    // has to be to another to be considered on the same lane
    void pointSort(float proximity);

    // Undo the rotation applied by the rotation filter
    void reverseRotation();

    // Visualization of the bot and top lanes
    void testLanes(const std::vector<Point> &bot, std::vector<Point> &top);

    // Print data regarding pointcloud
    void printPointCloud();

public:
    // Constructor
    Pointcloud(std::ifstream &file);

    // Getters
    std::shared_ptr<open3d::geometry::PointCloud> getPointcloud() const;
    std::vector<Point> getPoints() const;

    // Apply all filters.
    void applyFilters(int intensity, float angle, std::pair<float, float> distances,
        std::vector<float> slices, std::pair<float, int> cluster, float voxelSize, bool info);

    // Returns a 3 degree polynomial fitting
    std::pair<std::vector<double>, std::vector<double>> calculatePolynomial(const Point &separator, float proximity);
};