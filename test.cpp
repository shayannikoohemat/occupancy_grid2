//
// Created by NikoohematS on 6/6/2016.

// lower_bound/upper_bound example
#include <iostream>     // std::cout
#include <algorithm>    // std::lower_bound, std::upper_bound, std::sort
#include <vector>       // std::vector
#include "LaserPoints.h"

// for sorting points based on TimeTag
bool comp_attribute_laserpoints(const LaserPoint &p1, const LaserPoint &p2)
{
    return p1.Attribute(TimeTag)<p2.Attribute(TimeTag);
}

int main () {

    /// read laser points
    LaserPoints     surf_points, seg_laser_points;
    char            *laserFile;

    laserFile = (char *) "D://test//trash//sort_test.laser";
    surf_points.Read(laserFile);
    printf (" surfaces point size: %d \n ", surf_points.size());
    //printf ("\n sorting points, wait... \n ");
    //sort(surf_points.begin(),surf_points.end(),comp_attribute_laserpoints);  //expensive
    /// make new laserpoints from a segment
    seg_laser_points.ErasePoints();  // clear previous one if there is any
    seg_laser_points = surf_points.SelectTagValue(SegmentNumberTag, 6);  // expensive
    /// sort based on time_tag to derive the time stamp of the segment
    std::sort(seg_laser_points.begin(), seg_laser_points.end(), comp_attribute_laserpoints);
    vector<LaserPoint>::iterator p1_it = seg_laser_points.begin();
    LaserPoint point1 = *seg_laser_points.begin();

    double first_seg_point_timeTag  = seg_laser_points.begin() -> DoubleAttribute(TimeTag);
    double last_seg_point_timeTag   = (seg_laser_points.end()-1) -> DoubleAttribute(TimeTag);
    printf("First Time: %lf \n ", first_seg_point_timeTag);
    printf("Last Time: %lf \n ", last_seg_point_timeTag);
    seg_laser_points.Write("D://test//trash//segment.laser", false);


/*    double myints[] = {10.01,20.02,30.03,30.04,20.05,10.06,10.07,20.08};
    std::vector<double> v(myints,myints+8);           // 10 20 30 30 20 10 10 20

    std::sort (v.begin(), v.end());                // 10 10 10 20 20 20 30 30
    for (int i = 0; i<v.size(); i++)
    {
        std::cout << v[i] << '\n';
    }

    std::vector<double>::iterator low,up, low_iter;
    low=std::lower_bound (v.begin(), v.end(), 20.023); //          ^
    up= std::upper_bound (v.begin(), v.end(), 20.09); //                   ^

    std::cout << "lower_bound at position " << (low- v.begin()) << '\n';
    std::cout << "upper_bound at position " << (up - v.begin()) << '\n';
    std::cout << v[low-v.begin()-1] << '\n';
    std::cout << *low << '\n';
    std::cout << *up << '\n';*/

    return 0;
}