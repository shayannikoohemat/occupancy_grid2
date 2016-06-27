//
// Created by NikoohematS on 6/2/2016.
//


#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <Matrix3.h>
#include <vector>
#include <algorithm>
#include <sstream>
#include "LaserPoints.h"


using namespace std;

// for sorting points based on TimeTag
bool comp_attribute_laserpoints(LaserPoint p1, LaserPoint p2)
{
    return p1.Attribute(TimeTag)<p2.Attribute(TimeTag);
}

// Getting a laser point and finding and returning corresponding trajectory point based on TimeTag
LaserPoint traj2laserpoints(LaserPoint current_laserPoint, LaserPoints traj_points)
{
    LaserPoint current_trajectory;
    double timetag_matchedtrajectory, timetag_current_laserPoint;
    timetag_current_laserPoint = current_laserPoint.DoubleAttribute(TimeTag);

/*    if (!current_laserPoint.DoubleAttribute(TimeTag)) {
        timetag_current_laserPoint = current_laserPoint.DoubleAttribute(TimeTag);
    }else{
        printf ("  There is no TimeTag!!!  \n");
    }*/

    // sort traj_points with operator "<" based on TimeTag
    sort(traj_points.begin(), traj_points.end(), comp_attribute_laserpoints);

    std::vector<double> traj_points_timetag;
    std::vector<double>::iterator lower_traj;

    // generate a vector of traj_TimeTag
    for(int i=0; i<traj_points.size(); i++)
    {
        traj_points_timetag.push_back(traj_points[i].DoubleAttribute(TimeTag));
    }

    lower_traj = std::lower_bound(traj_points_timetag.begin(), traj_points_timetag.end(), timetag_current_laserPoint);

    cout << "lower bound at position: " << (lower_traj - traj_points_timetag.begin()) << endl;
    timetag_matchedtrajectory = traj_points_timetag[lower_traj - traj_points_timetag.begin() -1];

    current_trajectory = traj_points[lower_traj - traj_points_timetag.begin() -1];
    //current_trajectory.PrintAttributes();
    printf (" Points X: %.4f \n ", current_trajectory.X());
    printf (" Points Y: %.4f \n ", current_trajectory.Y());
    printf("Traj TimeTag:  %lf \n", timetag_matchedtrajectory);
    cout << "----------------------------" << endl;

    return(current_trajectory);

}  //end of function


