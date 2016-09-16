//
// Created by NikoohematS on 6/30/2016.
//

#include <iostream>
#include <cstdlib>
#include <Matrix3.h>
#include <vector>
#include "LaserPoints.h"
#include "Laservoxel.h"
#include "KNNFinder.h"
#include "Traj2LaserPoints.cpp"


using namespace std;

/// in the actual program we read just one laser points but currently we read two (1. wall points, 2. all points)
void raycasting(char* ascii_file_traj, char *laserFile, double traj_resolution) {

    LaserPoint traj2laserpoints(LaserPoint, LaserPoints, vector<double>);
    LaserVoxel vox(LaserPoints, double);
    bool comp_attribute_laserpoints(LaserPoint p1, LaserPoint p2); /// for sorting laser points based on TimeTag

    /// read laser points as Wall candidates
    LaserPoints surf_points;
    laserFile = (char *) "D://test//wall_crop.laser";
    surf_points.Read(laserFile);
    sort(surf_points.begin(),surf_points.end(),comp_attribute_laserpoints);

    /// read all laser points
    LaserPoints all_points;
    char *laserFile2;
    laserFile2 = (char *) "D://test//2015-10-13_11-46-46_Cloud_crop.laser";
    all_points.Read(laserFile2);

    /// read trajectory  ascii points
    std::FILE *ascii;
    ascii_file_traj = (char *) "D://test//wall_crop_traj.txt";

    printf ("Reading in ascii laser points...\n");
    ascii = Open_Compressed_File(ascii_file_traj, "r");
    if (!ascii) {
        fprintf(stderr, "Error opening input file %s\n", ascii_file_traj);
        exit(0);
    }
    /// read ascii file and transfer to laser format
    char    line[2048], *comma;
    fgets(line, 2048, ascii); ///ignore the first line as this is probably a header

    int             index1;
    double          value[21];
    LaserPoint      laserPoint;
    LaserPoints     traj_laser_points, temp_laser_points;
    do {
        if (fgets(line, 1024, ascii)) {
            index1++;
            /// Remove the comma's
            while ((comma = strchr(line, ',')) != NULL) *comma = ' ';
            /// Read the next line
            sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   value + 1, value + 2, value + 3, value + 4, value + 5,
                   value + 6, value + 7, value + 8, value + 9, value + 10,
                   value + 11, value + 12, value + 13, value + 14, value + 15,
                   value + 16, value + 17, value + 18, value + 19, value + 20);

            /// Copy the data to the laser point
            laserPoint.SetDoubleAttribute(TimeTag, value[1]);
            laserPoint.X() = value[2];
            laserPoint.Y() = value[3];
            laserPoint.Z() = value[4];

            //printf(" %lf \n", value[1]);

            temp_laser_points.push_back(laserPoint);
        }
        if (index1 == (index1 / 1000) * 1000) {
            /// 0.0 resolution for RemoveAlmostDoublePoints because we don't want to remove points in traj
            //TODO input trajectory resolution as variable
            temp_laser_points.RemoveAlmostDoublePoints(false, 0.0);
            printf(" %d / %d\r", traj_laser_points.size(), index1);
            fflush(stdout);
            traj_laser_points.AddPoints(temp_laser_points);
            temp_laser_points.ErasePoints();
        }
    }while (!feof(ascii));
    printf("\nRead %d points\n", traj_laser_points.size());
    traj_laser_points.Write("D:\\test\\raycasting\\traj_laser_points.laser", false);  // optional


/*    for (int m = 0; m<points.size(); m++)
    {
        curr_point = points[m];
        printf(" Current Point TimeTag %lf \n ", points[m].DoubleAttribute(TimeTag));
        //traj2laserpoints(curr_point, traj_laser_points);
    }*/

    //TODO sorting traj points should be done in the traj2laserpoint function
    /// sort trajectory points based on time tag
    sort(traj_laser_points.begin(), traj_laser_points.end(), comp_attribute_laserpoints);

    //TODO this should be done in the traj2laserpoint function
    /// generate a vector of traj_TimeTag
    std::vector<double>     traj_timetag_vec;
    for(int i=0; i<traj_laser_points.size(); i++)
    {
        traj_timetag_vec.push_back(traj_laser_points[i].DoubleAttribute(TimeTag));
    }



    ///************************************ segment processing **********************************///
    vector <int>            segment_numbers;
    vector <int>::iterator  segment_number;
    int                     minsizesegment = 100;

    segment_numbers = surf_points.AttributeValues((SegmentNumberTag));
    for (segment_number = segment_numbers.begin(); segment_number != segment_numbers.end(); segment_number++){

        LaserPoints         seg_laser_points;
        seg_laser_points.ErasePoints();
        /// make new laserpoints from a segment
        seg_laser_points = surf_points.SelectTagValue(SegmentNumberTag, *segment_number);

        /// Build Knn finder for segment points
        KNNFinder <LaserPoint> finder(seg_laser_points);  /// when should I destroy the KD-Tree ???

        if (seg_laser_points.size() > minsizesegment){
            printf (" segment size: %d \n ", seg_laser_points.size());

            Plane           seg_plane;
            Planes          seg_planes;
            /// generate a plane for the segment
            seg_plane = surf_points.FitPlane(*segment_number, *segment_number, SegmentNumberTag);
            seg_planes.push_back(seg_plane);

            Vector3D        projection_vector;
            //seg_laser_points.ProjectToPlane(projection_vector);  ???


            //TODO calculating convex hull should be user option
/*            /// generate the convexhull of the segment
            ObjectPoints convex_hull_points;
            LineTopologies convex_hull_lines;
            seg_laser_points.ConvexHull(convex_hull_points, convex_hull_lines, 5000);  // # just a number as max segment nr
            convex_hull_points.Write("D:\\test\\raycasting\\convex_hull_points.objpts"); // optional
            convex_hull_lines.Write("D:\\test\\raycasting\\convex_hull_lines.top", false); // optional*/

            /// generate voxels for the segment points
            double          vox_length = 0.10;
            LaserVoxel      vox(seg_laser_points, vox_length);
            LaserPoints     vox_centers, vox_all, vox_centers_filtered;
            /// export voxel-centers, centers of empty voxels have label 100
            vox_centers = vox.export_vox_centres();
            vox_all = vox.export_all();  /// export segment points
            //vox.statistics();

            vox_centers.Write("D:\\test\\raycasting\\vox_centers.laser", false);
            vox_all.Write("D:\\test\\raycasting\\vox_all.laser", false); // all segment points


            /// ******************************** Voxel Processing ***************************************************///
            /// select voxel-centers in a specific dist of the seg_plane
            for(int i=0; i<vox_centers.size(); i++)
            {
                double dist_plane_voxel;
                dist_plane_voxel = seg_plane.Distance(vox_centers[i]);

                if(fabs(dist_plane_voxel) < vox_length/2) // ???
                {
                    vox_centers_filtered.push_back(vox_centers[i]);
                }
            }
            vox_centers_filtered.Write("D:\\test\\raycasting\\vox_filterd.laser", false);

            /// select a voxel-center-filtered with label as empty voxel
            LaserPoints     kNN_points, empty_vox_centers, matched_trajectories;;
            vector<int>     indices;
            vector<double>  distances;

            empty_vox_centers = vox_centers_filtered.SelectTagValue(LabelTag, 100);
            empty_vox_centers.Write("D:\\test\\raycasting\\empty_vox_centers.laser", false);

/*
------------------------------------------------------------------------------------------------------------------------
                                 checking empty voxels for occlusion with raycasting
------------------------------------------------------------------------------------------------------------------------
*/
            bool raycastingTest = true;
            if (raycastingTest){
                /// select # nearest neighbors from seg-points for each empty-vox-center
                for (int i=0; i<empty_vox_centers.size(); i++)
                {
                    finder.FindKnn(empty_vox_centers[i], 5, distances, indices); // # = number of neighbor points
                    std::cout << "---------------------- " << i << " ----------------------" << endl ;  // debugger

                    /// looping thorough points of knn of an empty voxel
                    /// 1. find matched trajectories, 2. skip repeated trajectories, 3. build the ray
                    double      timetag_previous = 0.0;
                    double      traj_timetag_previous = 0.0;
                    for (int j = 0; j < indices.size(); ++j)
                    {
                        double  timetag;
                        double  dist;
                        kNN_points.push_back(seg_laser_points[indices[j]]); // debugger
                        timetag = seg_laser_points[indices[j]].DoubleAttribute(TimeTag);
                        printf("TIME_TAG: %lf \n ", timetag); // debugger
                        dist = distances[j];
                        //printf("Distance: %lf \n ", dist); // debugger

                        /// skip points with similar time_tag
                        if (timetag != timetag_previous){
                            /// find corresponding trajectory point for each point
                            LaserPoint current_traj, current_traj_temp;
                            double traj_timetag;

                            current_traj = traj2laserpoints(seg_laser_points[indices[j]], traj_laser_points, traj_timetag_vec);
                            traj_timetag = current_traj.DoubleAttribute(TimeTag);
                            /// skip repeated matched trajectory
                            if (current_traj.X() && current_traj.Y() != NULL){
                                if (traj_timetag != traj_timetag_previous){
                                    //TODO build a ray between this current_traj and voxel center
                                    matched_trajectories.push_back(current_traj);
                                }
                                traj_timetag_previous = traj_timetag;
                                std::cout << "---------------------------------" << endl; // debugger
                            }
                        }
                        timetag_previous = timetag;  /// saving timetag value in a variable for next repetition check
                    }
                }
                matched_trajectories.Write("D:\\test\\raycasting\\matched_trajectories.laser", false);
                kNN_points.Write("D:\\test\\raycasting\\kNN_points.laser", false);
            }
// ---------------------------------------------------------------------------------------------------------------------
        }
    }  // end of segments for loop
    ///************************************ end of segment processing ***********************************************///
}

