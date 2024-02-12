#pragma once

// C++ include
#include <Eigen/Dense>

/*
* Object that represents a point from the pointcloud.
* Contains x,y,z coordenates, an intensity value and a lidarBeam value.
*/
class Point 
{
private:
    float x;
    float y;
    float z;
    int intensity;
    int lidarBeam;

public:
    //Constructors
    Point() = default;
    Point(float x, float y, float z, int intensity = 0, int lidarBeam = 0);

    //Getters
    float getX() const;
    float getY() const;
    float getZ() const;
    Eigen::Vector3d getCoordinates() const;
    int getIntensity() const;

    //Setter
    void setCoordinates(float x, float y, float z);
};