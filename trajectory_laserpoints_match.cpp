//
// Created by NikoohematS on 5/24/2016.
//

#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <Matrix3.h>
#include <sstream>
#include "LaserPoints.h"


using namespace std;

// for sorting laser points based on TimeTag
bool comp_attribute_laserpoint(LaserPoint p1, LaserPoint p2)
{
    return p1.Attribute(TimeTag)<p2.Attribute(TimeTag);
}

void laserpoint_trajectory_match(char* ascii_file, char *laserFile, bool out_current, double traj_resolution)
{
    std::FILE *ascii;

    printf ("Reading in ascii laser points...\n");
    ascii = Open_Compressed_File(ascii_file, "r");
    if (!ascii) {
        fprintf(stderr, "Error opening input file %s\n", ascii_file);
        exit(0);
    }
    // read ascii file and transfer to laser format
    char    line[2048], *comma;
    fgets(line, 2048, ascii);//ignore the first line as this is probably a header

    int index1;
    double value[21];
    LaserPoint  laserPoint, pnt_first, pnt_last;
    LaserPoints traj_laser_points, laserPoints, temp_laser_points, laserPoints_time_signature;
    do {
        if (fgets(line, 1024, ascii)) {
            index1++;
            // Remove the comma's
            while ((comma = strchr(line, ',')) != NULL) *comma = ' ';
            // Read the next line
            sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   value + 1, value + 2, value + 3, value + 4, value + 5,
                   value + 6, value + 7, value + 8, value + 9, value + 10,
                   value + 11, value + 12, value + 13, value + 14, value + 15,
                   value + 16, value + 17, value + 18, value + 19, value + 20);

            // Copy the data to the laser point
            laserPoint.SetDoubleAttribute(TimeTag, value[1]);
            laserPoint.X() = value[2];
            laserPoint.Y() = value[3];
            laserPoint.Z() = value[4];

            //printf(" %lf \n", value[1]);

            temp_laser_points.push_back(laserPoint);
        }

        double traj_resolution;

        if (index1 == (index1 / 1000) * 1000) {
            // default point distance is 0.0 for RemoveAlmostDoublePoints
            // because we don't want to remove any point in traj
            temp_laser_points.RemoveAlmostDoublePoints(false, traj_resolution);
            printf(" %d / %d\r", traj_laser_points.size(), index1);
            fflush(stdout);
            traj_laser_points.AddPoints(temp_laser_points);
            temp_laser_points.ErasePoints();
        }
    }while (!feof(ascii));
    printf("\nRead %d points\n", traj_laser_points.size());

    /// read laser_points.laser
    laserPoints.Read(laserFile);

    /// sort laser points based on TimeTag attribute
    sort(laserPoints.begin(),laserPoints.end(),comp_attribute_laserpoint);

    LaserPoints::iterator iterator_time_tag;
    iterator_time_tag = laserPoints.begin();
    double pnt_first_time = iterator_time_tag->DoubleAttribute(TimeTag);
    iterator_time_tag = laserPoints.end();
    //double pnt_last_time = iterator_time_tag->DoubleAttribute(TimeTag); // code jumps out to stl_algo.h ???

    printf (" %lf \n", pnt_first_time);
    //printf (" %lf \n", pnt_last_time);
    //double pnt_first_time = laserPoints[0].DoubleAttribute(TimeTag);
    //double pnt_last_time = laserPoints[laserPoints.size()-1].DoubleAttribute(TimeTag);


    LaserPoints laserPoints_time_signed_current, trajpoint_current, new_trajectory;
    double time_tag_traj1, time_tag_traj2, time_tag_points;
    int cnt = 0;
    for (int i=0, j=1; i<traj_laser_points.size(), j<traj_laser_points.size(); i++, j++)
    {
        bool matched_points = false;
        time_tag_traj1 = traj_laser_points[i].DoubleAttribute(TimeTag);
        time_tag_traj2 = traj_laser_points[j].DoubleAttribute(TimeTag);
        printf("TIME_TAG: %lf \n ", time_tag_traj1);

        /// loop through the sorted points by time,
        // everytime the search starts from the last matched point index
        for (int m = cnt; m<laserPoints.size(); m++)
        {
            time_tag_points = laserPoints[m].DoubleAttribute(TimeTag);
//            printf(" %lf \n ", time_tag_points);
            if (time_tag_points >= time_tag_traj1 && time_tag_points < time_tag_traj2)
            {
                printf(" matched points: %lf \n ", time_tag_points);
                laserPoints[m].SetAttribute(ComponentNumberTag, i);  // index of trajectory point
                laserPoints[m].SetDoubleAttribute(TimeTrajTag, time_tag_traj1);

                // saving current points from i_th trajectory point
                if(out_current)
                {
                    laserPoints_time_signed_current.push_back(laserPoints[m]);
                }
                matched_points = true; // True when there are laser_points for a trajectory point
                cnt = m;
            }
        }

        ///  generates a new_trajectory based on matched laser_points
        if (matched_points)
        {
            //traj_laser_points[i].PrintAttributes();
            new_trajectory.push_back(traj_laser_points[i]);
        }

        //// -------  Writing output of the matching to the file -------------------------------------------------------
        if (out_current)
        {
            std::ostringstream sstream;
            sstream << fixed << time_tag_traj1;
            // convert double to string
            std::string time_tag_traj1_str = sstream.str();

            /// writing current laser points to file
            if (! laserPoints_time_signed_current.empty())
            {
                // convert string to char
                std::string current_laser_file = "D:\\test\\time_signature_out\\" + time_tag_traj1_str + ".laser";
                const char * out_laserpoints_current = current_laser_file.c_str();
                laserPoints_time_signed_current.Write(out_laserpoints_current, false);
            }
            laserPoints_time_signed_current.ErasePoints();

            /// writing current trajectory point to the file
            if (matched_points)
            {
                // convert string to char
                std::string current_traj_file = "D:\\test\\time_signature_out\\" + time_tag_traj1_str + "_traj.laser";
                const char * out_trajpoint_current = current_traj_file.c_str();
                trajpoint_current.push_back(traj_laser_points[i]);  // i is the current trajectory
                trajpoint_current.Write(out_trajpoint_current, false);
            }
            trajpoint_current.ErasePoints();
        }

        ////------------------------------------------------------------------------------------------------------------

    } //end of 1st for

    /// writing all laser points with traj properties to the file
    laserPoints_time_signature.ErasePoints();
    laserPoints_time_signature.AddPoints(laserPoints);
    printf (" %lf \n", laserPoints_time_signature[2000].DoubleAttribute(TimeTrajTag)); // random test
    laserPoints_time_signature.Write("D:\\test\\wall_crop_time_signed.laser", false);

    /// TODO
    /// Write a new trajectory file of matched trajectories with laser point
    new_trajectory.Write("D:\\test\\wall_crop_new_trajectory.laser", false);

}  //end of main void


