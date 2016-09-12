///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Authors:
// Date:
//////////////////////////////////////

#include "CollisionChecking.h"

// Intersect the point (x,y) with the set of rectangles.  If the point lies outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle>& obstacles)
{
    bool valid = true;
    for (uint i = 0; i < obstacles.size(); i++)
    {
    	double x_min = obstacles[i].x;
    	double x_max = obstacles[i].x + obstacles[i].width;
    	double y_min = obstacles[i].y;
    	double y_max = obstacles[i].y + obstacles[i].height;
	
    	if (x <= x_max && x>= x_min && y <= y_max && y >= y_min)
    	{
     	   valid = false;
    	}
    }
    return valid;
}

// Intersect a circle with center (x,y) and given radius with the set of rectangles.  If the circle
// lies outside of all obstacles, return true
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle>& obstacles)
{
    for (unsigned int i = 0; i < obstacles.size(); i++)
    {
    	double x_min = obstacles[i].x;
    	double x_max = obstacles[i].x + obstacles[i].width;
    	double y_min = obstacles[i].y;
    	double y_max = obstacles[i].y + obstacles[i].height;
    	double vertex_distances[4];
    	vertex_distances[0] = sqrt(pow(x-x_min,2) + pow(y-y_min,2));
    	vertex_distances[1] = sqrt(pow(x-x_min,2) + pow(y-y_max,2));
    	vertex_distances[2] = sqrt(pow(x-x_max,2) + pow(y-y_min,2));
    	vertex_distances[3] = sqrt(pow(x-x_max,2) + pow(y-y_max,2));
    	if ((x_min - radius <= x) && (x <= x_max + radius) && (y_min <= y) && (y <= y_max))
    	{
    		return false;
    	}
    	if ((x_min <= x) && (x <= x_max) && (y_min - radius <= y) && (y <= y_max + radius))
    	{
    		return false;
    	}
    	for (int i = 0; i < 4; i++)
    	{
    		if (vertex_distances[i] <= radius)
    		{
    			return false;
    		}
    	}
    }

    return true;
}

// Intersect a square with center at (x,y), orientation theta, and the given side length with the set of rectangles.
// If the square lies outside of all obstacles, return true
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle>& obstacles)
{
    // IMPLEMENT ME!

    return false;
}

// Add any custom debug / development code here.  This code will be executed instead of the
// statistics checker (Project2.cpp).  Any code submitted here MUST compile, but will not be graded.
void debugMode(const std::vector<Robot>& /*robots*/, const std::vector<Rectangle>& /*obstacles*/, const std::vector<bool>& /*valid*/)
{
}
