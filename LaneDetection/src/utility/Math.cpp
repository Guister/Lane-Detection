// Project include
#include "Math.hpp"

namespace Math
{   
    std::unordered_map<std::string, int> buildIntensityMap(const std::vector<Point> &points)
    {
        std::unordered_map<std::string, int> intensityMap 
        {
            {"0", 0},
            {"1", 0},
            {"2", 0},
            {"3", 0},
            {"4-5", 0},
            {"6-10", 0},
            {"10+", 0}
        };

        //Populate map according to each point's intensity
        for (const Point &p : points)
        {
            int intensity = p.getIntensity();
            if (intensity == 0)
                intensityMap["0"]++;
            else if (intensity == 1)
                intensityMap["1"]++;
            else if (intensity == 2)
                intensityMap["2"]++;
            else if (intensity == 3)
                intensityMap["3"]++;
            else if (intensity > 3 && intensity <= 5)
                intensityMap["4-5"]++;
            else if (intensity > 5 && intensity <= 10)
                intensityMap["6-10"]++;
            else if (intensity > 10)
            {
                intensityMap["+10"]++;
            }
            
        }
        return intensityMap;
    }

    double returnAverageIntensity(const std::vector<Point> &points)
    {
        int sum = 0;
        for (const Point &p : points)
        {
            sum += p.getIntensity();
        }
        double average = sum / points.size();
        return average;
    } 

    double toRadians(int angle)
    {
        return angle * M_PI / 180; //M_PI comes from cmath
    }

    Eigen::Matrix3d buildRotationMatrix(int angle)
    {
        //Rotation on the z axis;
        Eigen::Matrix3d rotationMatrix {
            {std::cos(toRadians(angle)), -std::sin(toRadians(angle)), 0},
            {std::sin(toRadians(angle)), std::cos(toRadians(angle)), 0},
            {0, 0, 1}
        };
        return rotationMatrix;
    }

        std::vector<double> polyfit(const std::vector<double> &t, const std::vector<double> &v, int order)
    {
        std::vector<double> coeff;
	    // Create Matrix Placeholder of size n x k, n= number of datapoints, k = order of polynomial, for exame k = 3 for cubic polynomial
	    Eigen::MatrixXd T(t.size(), order + 1);
	    Eigen::VectorXd V = Eigen::VectorXd::Map(&v.front(), v.size());
	    Eigen::VectorXd result;

	    // Check to make sure inputs are correct
	    assert(t.size() == v.size());
	    assert(t.size() >= order + 1);
	    // Populate the matrix
	    for(size_t i = 0 ; i < t.size(); ++i)
	    {
		    for(size_t j = 0; j < order + 1; ++j)
		    {
			    T(i, j) = pow(t.at(i), j);
		    }
	    }
	
	    // Solve for linear least square fit
	    result  = T.householderQr().solve(V);
	    coeff.resize(order+1);
	    for (int k = 0; k <= order; k++)
	    {
		    coeff[k] = result[k];
            std::cout << coeff[k] << ", ";
	    }
        std::cout << std::endl;
        return coeff;
    }
}