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
bool comp_attribute_laserpoints(const LaserPoint &p1, const LaserPoint &p2)
{
    return p1.Attribute(TimeTag)<p2.Attribute(TimeTag);
}
bool verbose;

/// Getting a laser point and finding and returning corresponding trajectory point based on TimeTag
LaserPoint traj2laserpoints(LaserPoint current_laserPoint, LaserPoints traj_points, vector<double> traj_points_timetag)
{
    LaserPoint          current_trajectory;
    double              timetag_current_laserPoint;
    timetag_current_laserPoint = current_laserPoint.DoubleAttribute(TimeTag);

    /// sort traj_points with operator "<" based on TimeTag
    //sort(traj_points.begin(), traj_points.end(), comp_attribute_laserpoints); //commented becasue is done in main function

    /// generate a vector of traj_TimeTag
/*    std::vector<double> traj_points_timetag;  //commented becasue is done in main function
    for(int i=0; i<traj_points.size(); i++)
    {
        traj_points_timetag.push_back(traj_points[i].DoubleAttribute(TimeTag));
    }*/

    std::vector<double>::iterator   lower_traj;
    lower_traj = std::lower_bound(traj_points_timetag.begin(), traj_points_timetag.end(), timetag_current_laserPoint);

    if ((lower_traj - traj_points_timetag.begin()) !=NULL){

        //cout << "lower bound at position: " << (lower_traj - traj_points_timetag.begin()) << endl; // debugger
        double        timetag_matchedtrajectory; //traj_timetag
        timetag_matchedtrajectory = traj_points_timetag[lower_traj - traj_points_timetag.begin() -1]; //debugger
        //printf("Traj TimeTag: %lf \n ", timetag_matchedtrajectory); //debugger

        ///current Point of matched-trajectory;
        //LaserPoint current_trajectory  // is defined in the function
        current_trajectory = traj_points[lower_traj - traj_points_timetag.begin() -1];
        double      traj_time_tag = current_trajectory.DoubleAttribute(TimeTag); //debugger
        //printf("Current Traj TIME_TAG: %lf \n ", traj_time_tag); //debugger
        //cout << "----------------------------" << endl;
        return(current_trajectory);
    }
    else{
        if (verbose) cout << "value is out of bound!!! Matched trajectory is NOT found." << endl;
        //cout << "----------------------------" << endl;
        EXIT_SUCCESS;
    }

}  //end of function


