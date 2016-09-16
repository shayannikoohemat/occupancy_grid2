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
    bool verbose;

    /// read laser points as Wall candidates
    LaserPoints surf_points;
    //laserFile = (char *) "D://test//surface_2_seg19.laser";
    laserFile = (char *) "D://test//surface_1.laser";
    surf_points.Read(laserFile);
    printf (" surfaces point size: %d \n ", surf_points.size());
    //printf ("\n sorting points, wait... \n ");
    //sort(surf_points.begin(),surf_points.end(),comp_attribute_laserpoints);  //expensive

    /// read all laser points
    LaserPoints all_points, subsampled_points;
    char *laserFile2;
    laserFile2 = (char *) "D://test//points_small_test.laser"; //points_crop_occlusion_raw_2_subsampled.laser";
    all_points.Read(laserFile2);
    printf (" laser points size: %d \n ", all_points.size());
    //printf ("\n Removing double points, wait... \n ");
    /// below filter removes a lot of points. Size change to 1/5th
    all_points.RemoveAlmostDoublePoints(true, 0.01);  // super expensive
    //printf (" laser points size after removing double points in 1cm : %d \n ", all_points.size());
    //printf ("\n sorting points, wait... \n ");
    //sort(all_points.begin(),all_points.end(),comp_attribute_laserpoints);
    //all_points.Write("D:\\test\\raycasting\\subsampled_points.laser", false);

    /// read trajectory laser points
    LaserPoints traj_laser_points;
    char *laserFile3;
    laserFile3 = (char *) "D://test//trajectory_2.laser";
    traj_laser_points.Read(laserFile3);
    printf (" trajectory size: %d \n ", traj_laser_points.size());
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

    /// build a vector of corresponding trajectory per laserpoint
    LaserPoints::iterator   lpIter;
    LaserPoint              current_traj0;
    LaserPoints             selected_current_trajectories;
    vector <int>            selected_lp_index;
    int                     index;
    std::clock_t            start_traj;
    double                  duration_traj;
    start_traj = std::clock();
    for (lpIter = all_points.begin(); lpIter != all_points.end(); lpIter++){
        current_traj0        = traj2laserpoints(*lpIter, traj_laser_points, traj_timetag_vec);
        /// if current_traj is not null then store it
        if ((current_traj0.X() && current_traj0.Y()) != NULL) {
            index = std::distance( all_points.begin(), lpIter );
            current_traj0.SetAttribute(ScalarTag, index);
            selected_current_trajectories.push_back(current_traj0);
        }
    }
    duration_traj = ( std::clock() - start_traj ) / (double) CLOCKS_PER_SEC;
    std::cout<<"traj processing time: "<< duration_traj << "s" << '\n';
    /// generate a vector of scalartegs as our index
    selected_lp_index = selected_current_trajectories.GetAttribute(ScalarTag);

    ///************************************ segment processing *************************************///
    //vector<int>             segment_numbers;
    vector <int>::iterator  segment_number;
    string                  segment_num_str;
    int                     seg_count = 0;
    int                     processed_points = 0;
    std::ostringstream      sstream;
    int                     minsizesegment = 100;
    int                     ray_count = 0;
    LaserPoints             labeledPoints, all_relabled_vox_centers, points_behindSurface;
    char                    output_prefix[500];
    string                  root_dir, subfolder, sub_directoy;
    root_dir                = "D:\\test\\raycasting";
    subfolder               = "surface_"; /////                             DON'T FORGET TO Change
    labeledPoints           = all_points;  // later we use it

    static const int arr[] = {19};  /// Define segments for processing
    //static const int arr[] = {5,13};
    vector<int> segment_numbers (arr, arr + sizeof(arr) / sizeof(arr[0]) );
    //segment_numbers = surf_points.AttributeValues((SegmentNumberTag));
    for (segment_number = segment_numbers.begin(); segment_number != segment_numbers.end(); segment_number++) {

        LaserPoints seg_laser_points;
        int occluded_cnt = 0;
        int opening_cnt = 0;
        int empty_cnt = 0;

        /// make new laserpoints from a segment
        seg_laser_points.ErasePoints();  // clear previous one if there is any
        seg_laser_points = surf_points.SelectTagValue(SegmentNumberTag, *segment_number);  // expensive

        /// just process segment with size of bigger than e.g. 100 points
        if (seg_laser_points.size() > minsizesegment){
            printf ("\n segment size: %d \n ", seg_laser_points.size());
            /// just select segments with WALL LabelTag (==4) for further processing
            if (seg_laser_points[0].Attribute(LabelTag) == 4) {  // we assume all points in the segment should have the same LabelTag
                /// print for the user, starting new segment
                seg_count++;
                cout << "========================= segment: " << *segment_number << "==========================" << endl;
                cout << "Ready to process " << seg_count << "-th surface out of " << segment_numbers.size() << endl;
                Sleep(3000);  // wait 3 s

                /// sort based on time_tag to derive the time stamp of the segment
                sort(seg_laser_points.begin(), seg_laser_points.end(), comp_attribute_laserpoints);
                double first_seg_point_timeTag  = seg_laser_points.begin() -> DoubleAttribute(TimeTag);
                double last_seg_point_timeTag   = (seg_laser_points.end()-1) -> DoubleAttribute(TimeTag);

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
                ///-------------------------------------------------------------------------------------///

                /// Build Knn finder for segment points
                //KNNFinder <LaserPoint> finder(seg_laser_points);  /// when should I destroy the KD-Tree ???

                /// make a directory per segment for surface processing
                sstream.str("");  // clear sstream
                sstream << *segment_number;  // convert to string
                segment_num_str     =sstream.str();
                /// sub_direcoty    =  "D:\\test\\raycasting" + "surface_" + segment_num_str
                sub_directoy = root_dir + "\\" + subfolder + segment_num_str;
                mkdir(sub_directoy.c_str());  // const *char
                strcpy (output_prefix, sub_directoy.c_str()); // e.g. "D:\test\raycasting\surf4\" as *char type

                Plane           seg_plane;
                Planes          seg_planes;
                /// generate a plane for the segment
                seg_plane = surf_points.FitPlane(*segment_number, *segment_number, SegmentNumberTag);
                seg_planes.push_back(seg_plane);

                //TODO calculating convex hull should be user option
                /*            /// generate the convexhull of the segment
                ObjectPoints convex_hull_points;
                LineTopologies convex_hull_lines;
                seg_laser_points.ConvexHull(convex_hull_points, convex_hull_lines, 5000);  // # just a number as max segment nr
                convex_hull_points.Write("D:\\test\\raycasting\\convex_hull_points.objpts"); // optional
                convex_hull_lines.Write("D:\\test\\raycasting\\convex_hull_lines.top", false); // optional*/

                /// generate voxels for the segment points
                vector< vector < vector < LaserPoints* > > > vox_filtered;
                double          vox_length = 0.10;
                LaserVoxel      vox(seg_laser_points, vox_length);
                LaserPoints     vox_centers, vox_all, vox_centers_filtered, vox0_f, vox0_c;
                /// export voxel-centers, centers of empty voxels have label 100
                vox_centers     = vox.export_vox_centres();
                vox_all         = vox.export_all();  /// export segment points
                vox_centers.SetAttribute(SegmentNumberTag, *segment_number);
                //vox.statistics();

                char            vox_centers_out[600];
                char            vox_all_out[600];

                strcpy (vox_centers_out, output_prefix);
                strcpy (vox_all_out, output_prefix);

                strcat(vox_centers_out, "\\vox_centers.laser");
                strcat(vox_all_out, "\\surface.laser");

                vox_centers.Write(vox_centers_out, false);
                vox_all.Write(vox_all_out, false); // all segment points


                /// ******************************** Voxel Processing ***************************************************///
                /// select voxel-centers in a specific distance of the seg_plane
                for(int i=0; i<vox_centers.size(); i++)
                {
                    double dist_plane_voxel;
                    dist_plane_voxel = seg_plane.Distance(vox_centers[i]);

                    if(fabs(dist_plane_voxel) < vox_length/2) // ???
                    {
                        vox_centers_filtered.push_back(vox_centers[i]);

                    }
                }

                char            vox_filtered_out[600];
                strcpy (vox_filtered_out, output_prefix);
                strcat(vox_filtered_out, "\\vox_filtered.laser");
                vox_centers_filtered.Write(vox_filtered_out, false);  // we write after relabeling
                vox_centers_filtered.Write(vox_filtered_out, false);
                KNNFinder <LaserPoint> finder(vox_centers_filtered); // knn for vox_filtered

                vox0_f.push_back(vox_centers_filtered[0]);
                vox0_f.Write("D:\\test\\raycasting\\vox0_f.laser", false);

                vox0_c.push_back(vox_centers[0]);
                vox0_c.Write("D:\\test\\raycasting\\vox0_c.laser", false);

                /// select a voxel-center-filtered with label as empty voxel
                LaserPoints     kNN_points, empty_vox_centers, matched_trajectories;;
                vector<int>     indices;
                vector<double>  distances;

                empty_vox_centers = vox_centers_filtered.SelectTagValue(LabelTag, 10);

                char            empty_vox_centers_out[600];
                strcpy (empty_vox_centers_out, output_prefix);
                strcat(empty_vox_centers_out, "\\empty_vox_centers.laser");
                empty_vox_centers.Write(empty_vox_centers_out, false);

/*
------------------------------------------------------------------------------------------------------------------------
                             Processing laser points for each surface
------------------------------------------------------------------------------------------------------------------------
*/
                ///  checking laser points and trajectory point for intersection with surface
                LaserPoints::iterator   pointIter;
                LaserPoints             current_traj, intersectionPoints, relabled_vox_centers;
                double                  traj_timetag, point_timetag;
                Line3D                  ray;
                Position3D              trajPos, pointPos;
                Vector3D                ray_direction;
                int                     ray_counter = 0, point_counter = 0;
                int outofrange_cnt = 0, inrange_cnt = 0;     //debugger

                relabled_vox_centers   = vox_centers_filtered; // later we use it for relabeling voxels
                for (pointIter = labeledPoints.begin(); pointIter != labeledPoints.end(); pointIter++){
                    //point_timetag       = pointIter -> DoubleAttribute(TimeTag);
                    point_counter++;
                    /// skip points which are out of sensor range (30m) for selected surface
                    double dist_ls2surf = seg_plane.Distance(*pointIter); //debugger
                    (dist_ls2surf > 30.00) ? outofrange_cnt++ : inrange_cnt++;        //debugger

                    /// this check barely is false
                    if (seg_plane.Distance(*pointIter) < 30.00){  // 30 m is the rang of laser sensor UTM 30LX

                        ray_counter = ray_counter++; // number of points for which ray is calculated
                        cout << "---------------------------" << ray_counter << "--------------------------" << endl;  //debugger
                        /// find corresponding trajectory point for current point
                        int index            = std::distance( labeledPoints.begin(), pointIter );
                        if (std::find(selected_lp_index.begin(), selected_lp_index.end(), index) != selected_lp_index.end()) {
                            /// current_traj is from laserpoints class but it contains just one point
                            current_traj = selected_current_trajectories.SelectTagValue(ScalarTag, index);
                            /// compare and print two timetags and check if they are correct
                            //traj_timetag = current_traj[0].DoubleAttribute(TimeTag);
                            //printf("TRAJ TIME0: %lf \n ", traj_timetag); // debugger
                            //printf("POINT TIME: %lf \n ", point_timetag); // debugger
                            /// if current_traj is not null then calculate the ray
                            if ((current_traj[0].X() && current_traj[0].Y()) != NULL) {
                                /// build the line between point and corresponding trajectory
                                ray_count       = ray_count++;
                                trajPos         = Position3D(current_traj[0]);   //???
                                pointPos        = Position3D(*pointIter);
                                ray             = Line3D (trajPos, pointPos);
                                ray_direction   = ray.Direction();

                                double                  ray_length, dist_trajTosurface;
                                Line3D                  trajsurf_3dline;
                                Vector3D                trajsurf_3dline_dir;
                                LaserPoint              intersectionPoint;
                                Position3D              intersectionPos, voxelPos;
                                int                     knnIndex;
                                double                  knnDistance;
                                double                  intersection_X,intersection_Y,intersection_Z;

                                /// check for the ray intersection with plane
                                if (IntersectLine3DPlane(ray,seg_plane, intersectionPos)){
                                    /// TODO handle error for rays parallel to the plane
                                    trajsurf_3dline     = Line3D(trajPos, intersectionPos);
                                    trajsurf_3dline_dir = trajsurf_3dline.Direction();

                                    intersection_X=intersectionPos.GetX();
                                    intersection_Y=intersectionPos.GetY();
                                    intersection_Z=intersectionPos.GetZ();

                                    /// check if the surface and laserpoint are in the same side of the trajectory
                                    if (ray_direction.DotProduct(trajsurf_3dline_dir) > 0){   /// u.v = u*v.cos(teta) should be >0
                                        //cout << "DOT PRODCUT: " << ray_direction.DotProduct(trajsurf_3dline_dir) << endl;
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
                                            knnDistance     = finder.FindDistance(intersectionPoint,1, EPS_DEFAULT);  // what distance
                                            printf ("intersection point DIST to the closest voxel: %lf \n", knnDistance);   // debugger

                                            /// /// Determine in which grid cell the intersection point falls
                                            int vox_x = ceil((intersection_X - seg_min_X)/vox_length);
                                            int vox_y = ceil((intersection_Y - seg_min_X)/vox_length);
                                            int vox_z = ceil((intersection_Z - seg_min_X)/vox_length);

                                            //vox_filtered [vox_x] [vox_y] [vox_z]

                                            voxelPos        = Position3D(vox_centers_filtered[knnIndex]);  // not used
                                            int voxelTag    = relabled_vox_centers[knnIndex].Attribute(LabelTag);
                                            printf ("VoxelTag: %d \n", voxelTag);  // debugger

                                            /// check if the laserpoint is close to the intersection point
                                            /// However points belong to the surface are supposed to be skipped in the beginning
                                            /// NOTE: This threshold can result in, points close to the wall become part of the wall
                                            if (fabs(ray_length - dist_trajTosurface) < 0.10) {  //same as surface growing dist to surf parameter
                                                /*point belongs to the surface*/
                                                relabled_vox_centers[knnIndex].SetAttribute(LabelTag, 11); // occupied
                                                cout << "label changed to OCCUPIED" << endl;
                                            }
                                            /// check for occlusion
                                            if(ray_length < dist_trajTosurface) {
                                                /*TODO intersectionPos is occluded*/
                                                /// if labeltag is already occupied(wall), occluded or opening(window) we don't change it
                                                if ( voxelTag <= 10) {
                                                    occluded_cnt = occluded_cnt++;
                                                    relabled_vox_centers[knnIndex].SetAttribute(LabelTag, 12); // occluded
                                                    cout << "label changed to OCCLUDED" << endl;
                                                }
                                            }
                                                /// check for opening
                                            else if(ray_length > dist_trajTosurface) {
                                                /*TODO intersectionPos is opening*/
                                                /// if labeltag is already occupied(wall) we don't change it
                                                if (10 == voxelTag || (12 == voxelTag)) {  //??? if empty or occluded // better practice: if not occupied
                                                    opening_cnt = opening_cnt++;
                                                    relabled_vox_centers[knnIndex].SetAttribute(LabelTag, 13); // opening
                                                    cout << "label changed to OPENING" << endl;
                                                    /// store points behind the surface for further analysis
                                                    points_behindSurface.push_back(*pointIter);
                                                }
                                            }
                                            else {                                                    // never should happen
                                                empty_cnt = empty_cnt++;
                                                cout << "label DID NOT change" << endl;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }  // end of points processing "forloop"
                cout << "------------ Processed Segment Statistics: " << *segment_number << " -------------" << endl;
                cout << "number of OPENING labels: " << opening_cnt << endl;
                cout << "number of OCCLUDED labels: " << occluded_cnt << endl;
                cout << "number of EMPTY labels without change: " << empty_cnt << endl;

                char            intersection_points_out[600];
                char            relabled_vox_centers_out[600];

                strcpy (intersection_points_out, output_prefix);
                strcpy (relabled_vox_centers_out, output_prefix);

                strcat(intersection_points_out, "\\intersection_points.laser");
                strcat(relabled_vox_centers_out, "\\relabled_vox_centers.laser");

                intersectionPoints.Write(intersection_points_out, false);
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
                cout << "# of points > 30 m : " << outofrange_cnt << endl;
                cout << "# of points in 30 m : " << inrange_cnt << endl;
                cout << "all laserpoints: " << all_points.size() << endl;
                cout << "processed points considering TimeTag: " << point_counter << endl;
                processed_points =+ point_counter;
            }
        }
    }  // end of segments "forloop"
    //all_relabled_vox_centers.Write("D:\\test\\raycasting\\all_relabled_vox_centers.laser", false);
    cout << "number of skipped_points considering timeTag: " <<  all_points.size() - processed_points << endl;

    ///************************************ end of segment processing ***********************************************///

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"processing time: "<< duration << "s" << '\n';

}

/*        std::ofstream out("D:\\test\\raycasting\\log.txt");
        std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
        std::cout.rdbuf(out.rdbuf()); //redirect std::cout to out.txt!
        std::cout.rdbuf(coutbuf); //reset to standard output again*/