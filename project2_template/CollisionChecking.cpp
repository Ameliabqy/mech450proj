///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Authors: Nathan Bucki, Amelia (Qingyun) Bian
// Date: 09/13/16
//////////////////////////////////////

#include "CollisionChecking.h"
#include <cmath>
#include <stdlib.h> 
#include <algorithm>

// Intersect the point (x,y) with the set of rectangles.  If the point lies outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle>& obstacles)
{
    bool valid = true;
    //Check if point is inside obstacle for each obstacle 
    for (unsigned int i = 0; i < obstacles.size(); i++)
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
    //Check each obstacle for collision
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
            //Collision with left/right side
    		return false;
    	}
    	if ((x_min <= x) && (x <= x_max) && (y_min - radius <= y) && (y <= y_max + radius))
    	{
            //Collision with top/botom side
    		return false;
    	}
    	for (int i = 0; i < 4; i++)
    	{
    		if (vertex_distances[i] <= radius)
    		{
                //Collision with corners
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
    double LocalCoordx[4] = {-sideLength/2, -sideLength/2, sideLength/2, sideLength/2};                            
    double LocalCoordy[4] = {-sideLength/2, sideLength/2, sideLength/2, -sideLength/2}; 
    double WorldCoordx[4];
    double WorldCoordy[4];
    for (unsigned int j = 0; j < 4; j++)
    {
        //Get coordinates of corners of robot in world frame
        double NewCoordx = LocalCoordx[j] * cos(theta) - LocalCoordy[j] * sin(theta) + x;
        double NewCoordy = LocalCoordx[j] * sin(theta) + LocalCoordy[j] * cos(theta) + y;
        WorldCoordx[j] = NewCoordx;
        WorldCoordy[j] = NewCoordy;
    }
    for (unsigned int p = 0; p < 4; p++)
    {
        //Loop through corner points on robot
        double x_A1 = WorldCoordx[p];
        double y_A1 = WorldCoordy[p];
        double x_B1;
        double y_B1;
        //Selecting line segment by selecting each end point 
        if (p == 3)
        {
            x_B1 = WorldCoordx[0];
            y_B1 = WorldCoordy[0];
        }
        else 
        {
            x_B1 = WorldCoordx[p + 1];
            y_B1 = WorldCoordy[p + 1];
        }

        for (unsigned int i = 0; i < obstacles.size(); i++)
        {
            //Loop through obstacles
	        double x_min = obstacles[i].x;
	        double x_max = obstacles[i].x + obstacles[i].width;
	        double y_min = obstacles[i].y;
	        double y_max = obstacles[i].y + obstacles[i].height;
            double ObCoordx[4] = {x_min, x_min, x_max, x_max};
            double ObCoordy[4] = {y_min, y_max, y_max, y_min};
            double x_B2;
            double y_B2;

            //Convert obstacle to robot coordinate system
            double x_min_R = (x_min - x)*cos(theta) + (y_min - y)*sin(theta);
            double y_min_R = (x - x_min)*sin(theta) + (y_min - y)*cos(theta);
            if(-sideLength/2 <= x_min_R && x_min_R <= sideLength/2 
                && -sideLength/2 <= y_min_R && y_min_R <= sideLength/2)
            {
                //Check if obstacle is inside of robot
                return false;
            }
            if(x_min <= x_A1 && x_A1 <= x_max && y_min <= y_A1 && y_A1 <= y_max)
            {
                //Robot is inside obastacle
                return false;
            }
            

            for (unsigned int q = 0; q < 4; q++)
            {
                //Loop through sides on obstacle
                double x_A2 = ObCoordx[q];
                double y_A2 = ObCoordy[q];
                if (q == 3)
                {
                    x_B2 = ObCoordx[0];
                    y_B2 = ObCoordy[0];
                }
                else 
                {
                    x_B2 = ObCoordx[q + 1];
                    y_B2 = ObCoordy[q + 1];
                }
                //Mathematical calculation of determining whether two line segments intersect. 
                //Check whether the line segment of the square robot and line segment of the obstacle intersect 
                double x_R1 = x_B1 - x_A1;
                double y_R1 = y_B1 - y_A1;
                double x_R2 = x_B2 - x_A2;
                double y_R2 = y_B2 - y_A2;
                double x_Adiff = x_A2 - x_A1;
                double y_Adiff = y_A2 - y_A1;
                double R2_cross_R1 = x_R2*y_R1 - x_R1*y_R2;
                double R1_cross_R2 = -R2_cross_R1;
                double Adiff_cross_R2 = x_Adiff*y_R2 - x_R2*y_Adiff;
                double Adiff_cross_R1 = x_Adiff*y_R1 - x_R1*y_Adiff;
                
                if(R2_cross_R1 != 0)
                {
                    double lambda_1 = Adiff_cross_R2/R1_cross_R2;
                    double lambda_2 = Adiff_cross_R1/R1_cross_R2;
                    if (0 <= lambda_1 && lambda_1 <= 1 && lambda_2 >= 0 && lambda_2 <= 1)
                    {
                        //Line segments intersect
                        return false;
                    }
                }
                
            }
        
        }
        
        
        
    }
    return true;
}

// Add any custom debug / development code here.  This code will be executed instead of the
// statistics checker (Project2.cpp).  Any code submitted here MUST compile, but will not be graded.
void debugMode(const std::vector<Robot>& /*robots*/, const std::vector<Rectangle>& /*obstacles*/, const std::vector<bool>& /*valid*/)
{

}
