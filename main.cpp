
#include <iostream>
#include <cstdlib>
#include <Matrix3.h>
#include <vector>
#include "InlineArguments.h"
#include "LaserPoints.h"
#include "laservoxel.h"
#include "KNNFinder.h"

using namespace std;


void PrintUsage()
{
    printf("Usage: raycasting -traj_points <trajectory laserpoints>\n");
    printf("                  -laser_points <input laserpoints>\n");
    printf("                  -out_current_laserpoints <export points and belonging trajectory>\n");
}


int main(int argc, char *argv[]) {

    InlineArguments *args = new InlineArguments(argc, argv);
    void laserpoint_trajectory_match(char* ascii_file, char *laserFile,
                                     bool out_current, double traj_resolution);

    // Check on required input files
    if (args->Contains("-usage") ||
        !args->Contains("-traj_points") ||
        !args->Contains("-laser_points"))
    {
        if (!args->Contains("-usage")) printf("Error: missing programme option.\n");
        PrintUsage();
        return EXIT_SUCCESS;
    }

    // for sorting laser points based on TimeTag
    bool comp_attribute_laserpoints(LaserPoint p1, LaserPoint p2);

    // Call the main function
/*    laserpoint_trajectory_match(args->String("-traj_points"), args->String("-laser_points"),
               args->String("-out_current_laserpoints"), args->Contains("-traj_resolution"));*/



    LaserPoint traj2laserpoints(LaserPoint, LaserPoints);
    LaserPoints points;
    LaserPoint curr_point;
    char *laserFile;
    laserFile = (char *) "D://test//wall_crop.laser";
    points.Read(laserFile);
    sort(points.begin(),points.end(),comp_attribute_laserpoints);

    std::FILE *ascii;

    char* ascii_file;
    ascii_file = (char *) "D://test//wall_crop_traj.txt";
    //char *laserFile;
    //laserFile = (char *) "D://test//wall_crop.laser";

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
    LaserPoint  laserPoint;
    LaserPoints traj_laser_points, temp_laser_points;
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
        if (index1 == (index1 / 1000) * 1000) {
            // 0.0001 resolution for RemoveAlmostDoublePoints because we don't want to remove points in traj
            temp_laser_points.RemoveAlmostDoublePoints(false, 0.0);
            printf(" %d / %d\r", traj_laser_points.size(), index1);
            fflush(stdout);
            traj_laser_points.AddPoints(temp_laser_points);
            temp_laser_points.ErasePoints();
        }
    }while (!feof(ascii));
    printf("\nRead %d points\n", traj_laser_points.size());


/*    for (int m = 0; m<points.size(); m++)
    {
        curr_point = points[m];
        printf(" Current Point TimeTag %lf \n ", points[m].DoubleAttribute(TimeTag));
        //traj2laserpoints(curr_point, traj_laser_points);
    }*/


    //TODO: select voxel centers that are within a distance from segment plane
    // distance should be less than voxel length distance
    ///////////// segment processing //////////////////////////
    vector <int>  segment_numbers;
    vector <int>::iterator segment_number;
    Plane seg_plane;
    Planes seg_planes;
    LaserPoints seg_laser_points, vox_centers, vox_all, vox_centers_filtered;
    int minsizesegment = 100;
    //LaserVoxel vox(LaserPoints ls, double vox_length);
    double dist_plane_voxel;
    double vox_length = 0.05;

    segment_numbers = points.AttributeValues((SegmentNumberTag));
    for (segment_number = segment_numbers.begin(); segment_number != segment_numbers.end(); segment_number++){
        seg_laser_points.ErasePoints();
        // make laserpoints from a segment
        seg_laser_points = points.SelectTagValue(SegmentNumberTag, *segment_number);
        KNNFinder <LaserPoint> finder(seg_laser_points);

        if (seg_laser_points.size() > minsizesegment){
            printf (" segment size: %d\n ", seg_laser_points.size());

            // generate a plane for the segment
            seg_plane = points.FitPlane(*segment_number, *segment_number, SegmentNumberTag);
            seg_planes.push_back(seg_plane);

            // generate voxels for the segment
            LaserVoxel vox(seg_laser_points, vox_length);
            // writes voxel-centers, centers of empty voxels have label 100
            vox_centers = vox.export_vox_centres();
            vox_all = vox.export_all();  // writes segment points
            //vox.statistics();
            vox_centers.Write("D:\\test\\vox_centers.laser", false); // just for test
            vox_all.Write("D:\\test\\vox_all.laser", false); // just for test

            // select voxel-centers in a specific dist of the seg_plane
            for(int i=0; i<vox_centers.size(); i++)
            {
                dist_plane_voxel = seg_plane.Distance(vox_centers[i]);
                //dist_plane_voxel = vox_centers[i].Distance(seg_plane);

                if(fabs(dist_plane_voxel) < vox_length/2)
                {
                    vox_centers_filtered.push_back(vox_centers[i]);
                }
            }
            vox_centers_filtered.Write("D:\\test\\vox_filterd.laser", false);

            // select a voxel-center-filtered with label as empty voxel
            // and assigning Time-Tag
            LaserPoints kNN_points, empty_vox_centers;
            vector<int> indices;
            vector<double> distances;

            empty_vox_centers = vox_centers_filtered.SelectTagValue(LabelTag, 100); /// ???
            empty_vox_centers.Write("D:\\test\\empty_vox_centers.laser", false); // just for check
            for (int i=0; i<empty_vox_centers.size(); i++)
            {
                finder.FindKnn(empty_vox_centers[i], 3, distances, indices); // # = number of neighbor points

                for (int j = 0; j < indices.size(); ++j)
                {
                    kNN_points.push_back(seg_laser_points[indices[j]]);
                }
            }
            kNN_points.Write("D:\\test\\kNN_points.laser", false);
        }
    }


    seg_planes.Write("D:\\test\\all_planes.top");


    ///////////////////////////////////////////////////////////

    LaserPoints rel_laserpoints;
    ObjectPoints rect_objpts;
    LineTopology rect_topology;
    // Do transformation for selected components
/*
    rel_laserpoints = points.Transform2RelativeCoordinates();

    rel_laserpoints.DeriveTIN();
    rel_laserpoints.DeriveDataBounds(0);
    rel_laserpoints.EnclosingRectangle(0.1, rect_objpts, rect_topology);*/

/*    ///initialize the LaserVoxel
    LaserVoxel vox(points,0.15);

    vox_centers = vox.export_vox_centres();
    vox_centers.Write("D:\\test\\wall_vox_centers.laser", false);*/


    return EXIT_SUCCESS;
}