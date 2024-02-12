// Project include
#include "Pointcloud.hpp"

// Constructor
Pointcloud::Pointcloud(std::ifstream &file)
{
    pointcloud = std::make_shared<open3d::geometry::PointCloud>();
    parseData(file);
    populatePointcloud();
    printPointCloud();
    open3d::visualization::DrawGeometries({pointcloud},"Pointcloud");
}

// Getters
std::shared_ptr<open3d::geometry::PointCloud> Pointcloud::getPointcloud() const
{
    return pointcloud;
}

std::vector<Point> Pointcloud::getPoints() const
{
    return points;
}

// Data manipulation
void Pointcloud::parseData(std::ifstream &file)
{
    std::vector<float> buffer;
    float f;
    //Add every value to our buffer
    while (file.read(reinterpret_cast<char*>(&f), sizeof(float)))
    {
        buffer.push_back(f);
    }
    //Each point is composed by 5 fields
    for (int i = 0; i < buffer.size(); i+=5)
    {
        points.emplace_back(buffer.at(i), buffer.at(i+1), buffer.at(i+2), buffer.at(i+3), buffer.at(i+4));
    }
}

void Pointcloud::populatePointcloud()
{
    pointcloud->points_.clear();
    for (const Point &p : points)
    {
        pointcloud->points_.push_back(p.getCoordinates());
    }
}

void Pointcloud::populatePointVector()
{
    points.clear();
    for (const auto &p : pointcloud->points_)
    {
        Point newP;
        newP.setCoordinates(p[0], p[1], p[2]);
        points.push_back(newP);
    }
}

void Pointcloud::update(DataType dataType)
{
    if (dataType == DataType::pointcloud) populatePointcloud();
    else if (dataType == DataType::data) populatePointVector();
}

// Filters
void Pointcloud::intensityFilter(int intensity)
{
    points.erase(std::remove_if(points.begin(), points.end(), [intensity](const Point &p) -> bool 
    {return p.getIntensity() < intensity;}), points.end());
}

void Pointcloud::rotationFilter(float angle)
{
    this->angle = angle;
    auto rotationMatrix = Math::buildRotationMatrix(angle);
    auto newPointcloud = pointcloud->Rotate(rotationMatrix, {0,0,0});
    *pointcloud = newPointcloud;
}

void Pointcloud::distanceFilter(float x, float y)
{
    auto filter = [x, y](const Point &p)
    {
        return (p.getX() < -x || p.getY() < -y ) ||
               (p.getX() > x || p.getY() > y );
    };

    points.erase(std::remove_if(points.begin(), points.end(), filter), points.end());
}

void Pointcloud::sliceFilter(float top, float bot, float innerTop, float innerBot)
{
    auto upSlice = [top](const Point &p) -> bool
    {
        return p.getY() > top;
    };
    auto bottomSlice = [bot](const Point &p) -> bool
    {
        return p.getY() < bot;
    };
    auto innerSlice = [innerTop, innerBot](const Point &p) -> bool
    {
        return p.getY() < innerTop && p.getY() > innerBot;
    };
    auto allSlices = [upSlice, bottomSlice, innerSlice](const Point &p) -> bool
    {
        return upSlice(p) || bottomSlice(p) || innerSlice(p);
    };

    points.erase(std::remove_if(points.begin(), points.end(), allSlices), points.end());
}

void Pointcloud::clusterDBSCAN(float density, int minPoints)
{
    std::vector<int> resultVector = pointcloud->ClusterDBSCAN(density, minPoints, true);
    //Points with value -1 are marked to be removed.
    for (int i = 0; i < resultVector.size(); i++)
    {
        if (resultVector.at(i) == -1)
        {
            points.erase(points.begin() + i);
        }
    }
}

void Pointcloud::voxelDownSample(double voxelSize)
{
    auto newPointcloud = pointcloud->VoxelDownSample(voxelSize);
    *pointcloud = *newPointcloud;
}

void Pointcloud::clean() 
{
    auto newPointcloud = pointcloud->RemoveNonFinitePoints();
    auto newPointcloud2 = newPointcloud.RemoveDuplicatedPoints();
    *pointcloud = newPointcloud2;
}

void Pointcloud::applyFilters(int intensity, float angle, std::pair<float, float> distances,
std::vector<float> slices, std::pair<float, int> cluster, float voxelSize, bool info)
{
    assert(slices.size() == 4);

    intensityFilter(intensity);
    update(DataType::pointcloud);
    if (info)
    {
        printPointCloud();
        open3d::visualization::DrawGeometries({pointcloud}, "Intensity filter Pointcloud");
    }

    rotationFilter(angle);
    update(DataType::data);
    if (info)
    {
        printPointCloud();
        open3d::visualization::DrawGeometries({pointcloud},"Rotation filter Pointcloud");
    }

    distanceFilter(distances.first, distances.second);
    update(DataType::pointcloud);
    if (info)
    {
        printPointCloud();
        open3d::visualization::DrawGeometries({pointcloud}, "Distance filter Pointcloud");
    }

    sliceFilter(slices.at(0), slices.at(1), slices.at(2), slices.at(3));
    update(DataType::pointcloud);
    if (info)
    {
        printPointCloud();
        open3d::visualization::DrawGeometries({pointcloud},"Slice filter Pointcloud");
    }

    clean();
    update(DataType::data);
    if (info)
    {
        printPointCloud();
        open3d::visualization::DrawGeometries({pointcloud}, "Clean filter Pointcloud");
    }

    clusterDBSCAN(cluster.first, cluster.second);
    update(DataType::pointcloud);
    if (info)
    {
        printPointCloud();
        open3d::visualization::DrawGeometries({pointcloud}, "ClusterDBSCAN Pointcloud");
    }

    voxelDownSample(voxelSize);
    update(DataType::data);
    if (info)
    {
        printPointCloud();
        open3d::visualization::DrawGeometries({pointcloud}, "Down Sampled Pointcloud");
    }

    printPointCloud();
    open3d::visualization::DrawGeometries({pointcloud}, "Final Pointcloud");
}

void Pointcloud::pointSort(float proximity)
{
    //Sort points within y coordinate proximity by x coordinate.
    auto sortComparison = [proximity](const Point &p1, const Point &p2) -> bool
    {
        float distance = std::abs(p1.getY() - p2.getY());
        if (distance < proximity) return p1.getX() < p2.getX();
        else return p1.getY() < p2.getY();
    };
    std::sort(points.begin(), points.end(), sortComparison);
}

void Pointcloud::reverseRotation()
{
    update(DataType::pointcloud);
    rotationFilter(-angle);
    update(DataType::data);
}

void Pointcloud::testLanes(const std::vector<Point> &bot, std::vector<Point> &top)
{
    auto pointcloudCopy = pointcloud;
    auto pointVectorCopy = points;
    points = bot;
    populatePointcloud();
    open3d::visualization::DrawGeometries({pointcloud}, "Bot Lane Pointcloud");
    points = top;
    populatePointcloud();
    open3d::visualization::DrawGeometries({pointcloud}, "Top Lane Pointcloud");
    pointcloud = pointcloudCopy;
    points = pointVectorCopy;
}

void Pointcloud::separateLanes(const Point &separator, float proximity,
 std::pair<std::vector<double>, std::vector<double>> &botLane,
 std::pair<std::vector<double>, std::vector<double>> &topLane)
    {
        //Add a point in between lanes that respects the sorting algorithm within the desired goal.
        points.push_back(separator);
        pointSort(proximity);

        //Since we applied a rotation filter we need to undo it to get our true coordinates.
        reverseRotation();

        //Since the rotation is done on the z axis, the separator z value will remain unmodified.
        auto it = std::find_if(points.begin(), points.end(), [&separator](const Point &point)-> bool
        {return point.getZ() == separator.getZ();});

        //We don't want the separator Point (it), so create the vectors with it in between (excluding it)
        std::vector<Point> bot(points.begin(), it); 
        std::vector<Point> top(it + 1, points.end()); 
        testLanes(bot, top);

        //Separate the coordinates for the lane fitting.
        for (const Point &p : bot)
        {
            botLane.first.push_back(p.getX());
            botLane.second.push_back(p.getY());
        }
        for (const Point &p :top)
        {
            topLane.first.push_back(p.getX());
            topLane.second.push_back(p.getY());
        }
    }

std::pair<std::vector<double>, std::vector<double>> Pointcloud::calculatePolynomial(const Point &separator, float proximity)
{
    std::pair<std::vector<double>, std::vector<double>> botLane, topLane;
    separateLanes(separator, proximity, botLane, topLane);

    auto topAnswer = Math::polyfit(topLane.first, topLane.second, 3);
    auto bottomAnswer = Math::polyfit(botLane.first, botLane.second, 3);
    return std::make_pair(topAnswer, bottomAnswer);
}

//From open3d examples
void Pointcloud::printPointCloud()
{
    using namespace open3d;

    bool pointCloud_has_normal = pointcloud->HasNormals();
    utility::LogInfo("Pointcloud has {} points.",
                     pointcloud->points_.size());

    auto x = pointcloud->points_.size();

    Eigen::Vector3d min_bound = pointcloud->GetMinBound();
    Eigen::Vector3d max_bound = pointcloud->GetMaxBound();
    utility::LogInfo(
            "Bounding box is: ({:.4f}, {:.4f}, {:.4f}) - ({:.4f}, {:.4f}, "
            "{:.4f})",
            min_bound(0), min_bound(1), min_bound(2), max_bound(0),
            max_bound(1), max_bound(2));

    /*for (size_t i = 0; i < pointcloud->points_.size(); i++) {
        if (pointCloud_has_normal) {
            const Eigen::Vector3d &point = pointcloud->points_[i];
            const Eigen::Vector3d &normal = pointcloud->normals_[i];
            utility::LogInfo("{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}",
                             point(0), point(1), point(2), normal(0), normal(1),
                             normal(2));
        } else {
            const Eigen::Vector3d &point = pointcloud->points_[i];
            utility::LogInfo("{:.6f} {:.6f} {:.6f}", point(0), point(1),
                             point(2));
        }
    }*/
    utility::LogInfo("End of the list.");
}
