// Project include
#include "Point.hpp"

// Constructor
Point::Point(float x, float y, float z, int intensity, int lidarBeam) :
x(x), y(y), z(z), intensity(intensity), lidarBeam(lidarBeam) {}

// Getters
float Point::getX() const
{
    return x;
}
float Point::getY() const
{
    return y;
}
float Point::getZ() const
{
    return z;
}
Eigen::Vector3d Point::getCoordinates() const
{
    return {x, y, z};
}
int Point::getIntensity() const
{
    return intensity;
}

// Setter
void Point::setCoordinates(float x, float y, float z)
{
    this->x = x;
    this->y = y;
    this->z = z;
}
