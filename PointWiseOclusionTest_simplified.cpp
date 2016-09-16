//
// Created by NikoohematS on 6/30/2016.
//

#include <iostream>
#include <windows.h>
#include <string>
#include <sys/stat.h>
#include <cstdlib>
#include <Matrix3.h>
#include <vector>
#include <sstream>
#include <ctime>
#include "LaserPoints.h"
#include "Laservoxel.h"
#include "KNNFinder.h"
#define EPS_DEFAULT 1e-4

using namespace std;


/// in the actual program we read just one laser points but currently we read two (1. wall points, 2. all points)
void PointWiseOcclusionTest(char* ascii_file_traj, char *laserFile, double traj_resolution) {

    std::clock_t start;
    double duration;

    start = std::clock();
    LaserPoint traj2laserpoints(LaserPoint current_laserPoint, LaserPoints traj_points, vector<double> traj_points_timetag);
    LaserVoxel vox(LaserPoints, double);
    bool comp_attribute_laserpoints(const LaserPoint &p1, const LaserPoint &p2); /// for sorting laser points based on TimeTag
    bool verbose =1;
    bool folder_out=1;


    /// read laser points as Wall candidates
    LaserPoints surf_points;
    //laserFile = (char *) "D://test//surface_2_seg19.laser";
    laserFile = (char *) "D://test//surface_4_thinned.laser";  //
    //laserFile = (char *) "D://test//surface_occlusionpointwise_wall_corrected_thinned.laser";
    surf_points.Read(laserFile);
    printf (" surfaces point size: %d \n ", surf_points.size());

    /// read all laser points
    LaserPoints all_points, subsampled_points;
    char *laserFile2;
    laserFile2 = (char *) "D://test//points_for_surf3_thinned_1.5m.laser"; //
    //laserFile2 = (char *) "D://test//points_crop_occlusion_raw_4_thinned_100k_segmented_nofloorceiling.laser";
    all_points.Read(laserFile2);
    printf (" laser points size: %d \n ", all_points.size());

    /// read trajectory laser points
    LaserPoints traj_laser_points;
    char *laserFile3;
    laserFile3 = (char *) "D://test//trajectory3_sub10cm.laser";
    //laserFile3 = (char *) "D://test//trajectory_occlusionpointwise_wall_thinned.laser";
    traj_laser_points.Read(laserFile3);
    printf (" trajectory size: %d \n ", traj_laser_points.size());
    /// sort trajectory points based on time tag
    //sort(traj_laser_points.begin(), traj_laser_points.end(), comp_attribute_laserpoints);

    /// generate a vector of traj_TimeTag
    std::vector<double>     traj_timetag_vec;
    for(int i=0; i<traj_laser_points.size(); i++)
    {
        traj_timetag_vec.push_back(traj_laser_points[i].DoubleAttribute(TimeTag));
    }

    /// build a vector of corresponding trajectory per laserpoint
    LaserPoints::iterator   lpIter;
    LaserPoint              current_traj0;
    LaserPoints             selected_current_trajectories;
    vector <int>            selected_lp_index;
    int                     index;
    std::clock_t            start_traj;
    double                  duration_traj;

/*    start_traj = std::clock();
    printf ("\n ...matching points and trajectories:, wait... \n ");
    for (lpIter = all_points.begin(); lpIter != all_points.end(); lpIter++){
        current_traj0        = traj2laserpoints(*lpIter, traj_laser_points, traj_timetag_vec);
        /// if current_traj is not null then store it
        if ((current_traj0.X() && current_traj0.Y()) != NULL) {
            index = std::distance( all_points.begin(), lpIter );
            current_traj0.SetAttribute(ScalarTag, index);
            selected_current_trajectories.push_back(current_traj0);  // this generates repeated trajectories
        }
    }
    duration_traj = ( std::clock() - start_traj ) / (double) CLOCKS_PER_SEC;
    std::cout<<"traj processing time: "<< duration_traj << "s" << '\n';
    /// generate a vector of scalartegs as our index
    selected_lp_index = selected_current_trajectories.GetAttribute(ScalarTag);*/

    ///************************************ segment processing *************************************///
    //vector<int>             segment_numbers, label_numbers;
    vector <int>::iterator  segment_number;
    string                  segment_num_str;
    int                     seg_count = 0;
    int                     processed_points = 0;
    std::ostringstream      sstream;
    int                     minsizesegment = 1500;
    int                     ray_count = 0;
    LaserPoints             all_relabled_vox_centers, points_behindSurface;
    char                    output_prefix[500];
    string                  root_dir, subfolder, sub_directoy;
    root_dir                = "D:\\test\\raycasting";
    subfolder               = "surface_"; /////     default "surface_"

    std::clock_t start_seg;
    double duration_seg;

    //static const int arr[] = {18,19,20,31,32,47,7,30,39,24};  /// Define segments for processing
    static const int arr[] = {2,3};
    vector<int> segment_numbers (arr, arr + sizeof(arr) / sizeof(arr[0]) );
    //segment_numbers = surf_points.AttributeValues((SegmentNumberTag));
    for (segment_number = segment_numbers.begin(); segment_number != segment_numbers.end(); segment_number++) {

        LaserPoints seg_laser_points;
        //int occluded_cnt = 0;
        //int opening_cnt = 0;
        //int empty_cnt = 0;

        /// make new laserpoints from a segment
        seg_laser_points.ErasePoints();  // clear previous one if there is any
        seg_laser_points = surf_points.SelectTagValue(SegmentNumberTag, *segment_number);

        /// just process segment with size of bigger than e.g. 100 points
        if (seg_laser_points.size() > minsizesegment){
            printf ("\n segment size: %d \n ", seg_laser_points.size());
            /// just select segments with WALL LabelTag (==4) for further processing
            if (seg_laser_points[0].Attribute(LabelTag) == 4) {  // we assume all points in the segment should have the same LabelTag
                /// print for the user, starting new segment

                start_seg = std::clock();
                seg_count++;
                cout << "========================= segment: " << *segment_number << "==========================" << endl;
                cout << "Ready to process " << seg_count << "-th surface out of " << segment_numbers.size() << endl;
                //Sleep(3000);  // wait 3 s

                KNNFinder <LaserPoint> find_duplicates(seg_laser_points);

                ///min and max value in real X Y Z (bounding box) of the segment
                double                  seg_min_X, seg_min_Y, seg_min_Z;
                double                  seg_max_X, seg_max_Y, seg_max_Z;
                ///get the databounds from the segment laserpoints
                // this is for later use in intersection with trajectory
                DataBoundsLaser db=seg_laser_points.DeriveDataBounds(0);
                seg_min_X=db.Minimum().GetX();
                seg_min_Y=db.Minimum().GetY();
                seg_min_Z=db.Minimum().GetZ();

                seg_max_X=db.Maximum().GetX();
                seg_max_Y=db.Maximum().GetY();
                seg_max_Z=db.Maximum().GetZ();

                /// make a directory per segment for surface processing
                if (folder_out) {
                    sstream.str("");  // clear sstream
                    sstream << *segment_number;  // convert to string
                    segment_num_str     =sstream.str();
                    /// sub_direcoty    =  "D:\\test\\raycasting" + "surface_" + segment_num_str
                    sub_directoy = root_dir + "\\" + subfolder + segment_num_str;
                    mkdir(sub_directoy.c_str());  // const *char
                    strcpy (output_prefix, sub_directoy.c_str()); // e.g. "D:\test\raycasting\surf4\" as *char type
                }

                Plane           seg_plane;
                Planes          seg_planes;
                /// generate a plane for the segment
                seg_plane = surf_points.FitPlane(*segment_number, *segment_number, SegmentNumberTag);
                seg_planes.push_back(seg_plane);

                /// generate voxels for the segment points
                double          vox_length = 0.15;
                LaserVoxel      vox(seg_laser_points, vox_length);
                LaserPoints     vox_centers, vox_centers_filtered;
                /// export voxel-centers, centers of empty voxels have label 100
                vox_centers     = vox.export_vox_centres();
                vox_centers.SetAttribute(SegmentNumberTag, *segment_number);
                //vox.statistics();

                if (folder_out){
                    char            vox_centers_out[600];
                    char            seg_points_out[600];


                    strcpy (vox_centers_out, output_prefix);
                    strcpy (seg_points_out, output_prefix);

                    strcat(vox_centers_out, "\\vox_centers.laser");
                    strcat(seg_points_out, "\\surface.laser");

                    vox_centers.Write(vox_centers_out, false);
                    seg_laser_points.Write(seg_points_out, false); // all segment points
                }

                /// ******************************** Voxel Processing ***************************************************///
                /// select voxel-centers in a specific distance of the seg_plane
                for(int i=0; i<vox_centers.size(); i++)
                {
                    double dist_plane_voxel;
                    dist_plane_voxel = seg_plane.Distance(vox_centers[i]);

                    if(fabs(dist_plane_voxel) < vox_length/2) {  /// vox_length/2 ???
                        vox_centers_filtered.push_back(vox_centers[i]);
                    }
                }

                char            vox_filtered_out[600];
                strcpy (vox_filtered_out, output_prefix);
                strcat(vox_filtered_out, "\\vox_filtered.laser");
                vox_centers_filtered.Write(vox_filtered_out, false);  // we write it also after relabeling

                /// Build KNN for vox_centers_filtered
                KNNFinder <LaserPoint> finder(vox_centers_filtered); // we use this later for finding closest intersection points

/*                /// select a voxel-center-filtered with label as empty voxel
 * // commented as all centers labeled as unoccupied
                LaserPoints     kNN_points, empty_vox_centers, matched_trajectories;;
                vector<int>     indices;
                vector<double>  distances;

                empty_vox_centers = vox_centers_filtered.SelectTagValue(LabelTag, 10);

                char            empty_vox_centers_out[600];
                strcpy (empty_vox_centers_out, output_prefix);
                strcat(empty_vox_centers_out, "\\empty_vox_centers.laser");
                empty_vox_centers.Write(empty_vox_centers_out, false);*/

/*
------------------------------------------------------------------------------------------------------------------------
                             Processing laser points for each surface
------------------------------------------------------------------------------------------------------------------------
*/
                ///  checking laser points and trajectory point for intersection with surface
                LaserPoints::iterator   pointIter;
                LaserPoints             intersectionPoints, relabled_vox_centers, duplicatepoints, current_traj;
                Line3D                  ray;
                Position3D              trajPos, pointPos;
                Vector3D                ray_direction;
                double                  duplicate_Distance;

                relabled_vox_centers   = vox_centers_filtered; // later we use it for relabeling voxels
                printf ("\n ...labeling voxels:, wait... \n ");
                for (pointIter = all_points.begin(); pointIter != all_points.end(); pointIter++){
                    //point_timetag       = pointIter -> DoubleAttribute(TimeTag);

                    // TODO: Skip segment points here by knnfinder or removeduoblepoints
                    duplicate_Distance     = find_duplicates.FindDistance(*pointIter,1, EPS_DEFAULT);
                    //if (duplicate_Distance < 0.001) duplicatepoints.push_back(*pointIter); //debugger
                    if (duplicate_Distance > 0.001) {
                        //double dist_ls2surf = seg_plane.Distance(*pointIter); //debugger
                        //(dist_ls2surf > 30.00) ? outofrange_cnt++ : inrange_cnt++;        //debugger
                        /// skip points which are out of sensor range (30m) for selected surface
                        /// this check barely is false
                        if (seg_plane.Distance(*pointIter) < 30.00){  // 30 m is the rang of laser sensor UTM 30LX

                            //ray_counter = ray_counter++; // number of points for which ray is calculated
                            //cout << "---------------------------" << ray_counter << "--------------------------" << endl;  //debugger
                            /// find corresponding trajectory point for current point
                            //int index            = std::distance( all_points.begin(), pointIter );
                            //if (std::find(selected_lp_index.begin(), selected_lp_index.end(), index) != selected_lp_index.end()) {
                                /// current_traj is from laserpoints class but it contains just one point
                                current_traj0        = traj2laserpoints(*pointIter, traj_laser_points, traj_timetag_vec);
                                //current_traj = selected_current_trajectories.SelectTagValue(ScalarTag, index);

                                /// if current_traj is not null then calculate the ray
                                if ((current_traj0.X() && current_traj0.Y()) != NULL) {
                                    /// build the line between point and corresponding trajectory
                                    //ray_count       = ray_count++;
                                    trajPos         = Position3D(current_traj0);   //???
                                    pointPos        = Position3D(*pointIter);
                                    ray             = Line3D (trajPos, pointPos);
                                    ray_direction   = ray.Direction();

                                    double                  ray_length, dist_trajTosurface;
                                    Line3D                  trajsurf_3dline;
                                    Vector3D                trajsurf_3dline_dir;
                                    LaserPoint              intersectionPoint;
                                    Position3D              intersectionPos;
                                    int                     knnIndex;
                                    double                  knnDistance;
                                    double                  intersection_X,intersection_Y,intersection_Z;

                                    /// check for the ray intersection with plane
                                    if (IntersectLine3DPlane(ray,seg_plane, intersectionPos)){  // returns intersectionPos
                                        /// TODO handle error for rays parallel to the plane
                                        trajsurf_3dline     = Line3D(trajPos, intersectionPos);
                                        trajsurf_3dline_dir = trajsurf_3dline.Direction();

                                        intersection_X=intersectionPos.GetX();
                                        intersection_Y=intersectionPos.GetY();
                                        intersection_Z=intersectionPos.GetZ();
                                        void relabeling (double ,double , double ,LaserPoint ,int ,bool );
                                        /// check if the surface and laserpoint are in the same side of the trajectory
                                        if (ray_direction.DotProduct(trajsurf_3dline_dir) > 0){   /// u.v = u*v.cos(teta) should be >0
                                            /// Check if intersection point falls in the boundary of the segment points
                                            if ( seg_min_X < intersection_X && intersection_X < seg_max_X &&
                                                 seg_min_Y < intersection_Y && intersection_Y < seg_max_Y &&
                                                 seg_min_Z < intersection_Z && intersection_Z < seg_max_Z ) {

                                                ray_length          = pointPos.Distance(trajPos);
                                                dist_trajTosurface  = trajPos.Distance(intersectionPos);
                                                intersectionPoint   = LaserPoint(intersectionPos);
                                                intersectionPoints.push_back(intersectionPoint);  // debugger

                                                /// Determine in which grid cell the intersection point falls and in what distance
                                                //finder.FindKnn(intersectionPoint, 1, knnDistance, knnIndex);
                                                knnIndex        = finder.FindIndex(intersectionPoint,1, EPS_DEFAULT);  // which index
                                                //knnDistance     = finder.FindDistance(intersectionPoint,1, EPS_DEFAULT);  // what distance
                                                //printf ("intersection point DIST to the closest voxel: %lf \n", knnDistance);   // debugger

                                                /// /// Determine in which grid cell the intersection point falls
                                                //int vox_x = ceil((intersection_X - seg_min_X)/vox_length);
                                                //int vox_y = ceil((intersection_Y - seg_min_X)/vox_length);
                                                //int vox_z = ceil((intersection_Z - seg_min_X)/vox_length);
                                                //vox_filtered [vox_x] [vox_y] [vox_z]

                                                int     voxelTag    = relabled_vox_centers[knnIndex].Attribute(LabelTag);
                                                bool    occupied    = false,
                                                        opening     = false;
                                                if (11 == voxelTag) occupied = true;
                                                if (13 == voxelTag) opening  = true;

                                                if (!occupied) {
                                                    /// first check: check if the laserpoint is close to the intersection
                                                    /// NOTE: This threshold can result in, points close to the wall become part of the wall
                                                    if (fabs(ray_length - dist_trajTosurface) <= 0.10) {  //same as surface growing dist to surf parameter
                                                        if (!opening) {
                                                            //*point belongs to the surface*//*
                                                            relabled_vox_centers[knnIndex].SetAttribute(LabelTag, 11); // occupied
                                                            occupied = true;
                                                            //cout << "label changed to OCCUPIED" << endl;
                                                        }
                                                    }
                                                        /// second check: check for occlusion
                                                    else if(ray_length < dist_trajTosurface) {
                                                        /// if labeltag is already occupied, occluded or opening(window) we don't change it
                                                        if ( voxelTag <= 10) {
                                                            //occluded_cnt = occluded_cnt++;
                                                            relabled_vox_centers[knnIndex].SetAttribute(LabelTag, 12); // occluded
                                                            //cout << "label changed to OCCLUDED" << endl;
                                                        }
                                                    }
                                                        ///third check:  check for opening
                                                    else if(ray_length > dist_trajTosurface) {
                                                        if (!opening) {
                                                            //opening_cnt = opening_cnt++;
                                                            relabled_vox_centers[knnIndex].SetAttribute(LabelTag, 13); // opening
                                                            //cout << "label changed to OPENING" << endl;
                                                            /// store points behind the surface for further analysis
                                                            points_behindSurface.push_back(*pointIter);
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            //}
                        }
                    }
                }  // end of points processing "forloop"
                //cout << "------------ Processed Segment Statistics: " << *segment_number << " -------------" << endl;
                //cout << "number of OPENING labels: " << opening_cnt << endl;
                //cout << "number of OCCLUDED labels: " << occluded_cnt << endl;
                //cout << "number of EMPTY labels without change: " << empty_cnt << endl;

                //char            intersection_points_out[600];
                char            relabled_vox_centers_out[600];

                //strcpy (intersection_points_out, output_prefix);
                strcpy (relabled_vox_centers_out, output_prefix);

                //strcat(intersection_points_out, "\\intersection_points.laser");
                strcat(relabled_vox_centers_out, "\\relabled_vox_centers.laser");

                //intersectionPoints.Write(intersection_points_out, false);
                relabled_vox_centers.Write(relabled_vox_centers_out, false);

                all_relabled_vox_centers.AddPoints(relabled_vox_centers);
                all_relabled_vox_centers.Write("D:\\test\\raycasting\\all_relabled_vox_centers.laser", false);
                vox_centers_filtered.ErasePoints(); //???
                relabled_vox_centers.ErasePoints();

                char    points_behindSurface_out[600];
                strcpy (points_behindSurface_out, output_prefix);
                strcat (points_behindSurface_out, "\\points_behindSurface.laser");
                points_behindSurface.Write(points_behindSurface_out, false);
                points_behindSurface.ErasePoints();
                /// debugger
                cout << "all laserpoints: " << all_points.size() << endl;
                //duplicatepoints.Write("D:\\test\\raycasting\\surf_test_3\\dups.laser", false);
            }
        }
        duration_seg = ( std::clock() - start_seg ) / (double) CLOCKS_PER_SEC;
        std::cout<<"segment processing time: "<< duration_seg << "s" << '\n';
    }  // end of segments "forloop"
    //all_relabled_vox_centers.Write("D:\\test\\raycasting\\all_relabled_vox_centers.laser", false);

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"total processing time: "<< duration/60 << "m" << '\n';
}

/*        std::ofstream out("D:\\test\\raycasting\\log.txt");
        std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
        std::cout.rdbuf(out.rdbuf()); //redirect std::cout to out.txt!
        std::cout.rdbuf(coutbuf); //reset to standard output again*/

/// not used
void relabeling (double ray_l,double dist_TrajSurface, double threshold, LaserPoint v, int vTag, bool open)
{
    if ((fabs(ray_l - dist_TrajSurface)) <= threshold){
        if (!open) v.SetAttribute(LabelTag, 11);    //occupied
    }
    else if (ray_l < dist_TrajSurface) {
        if (vTag <= 10) v.SetAttribute(LabelTag, 12);
    } //occluded
    else if (ray_l > dist_TrajSurface) {
        if (!open)      v.SetAttribute(LabelTag, 13);
    } //opening
}