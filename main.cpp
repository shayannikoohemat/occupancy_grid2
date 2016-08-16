
#include <iostream>
#include <cstdlib>
#include <vector>
#include "InlineArguments.h"
#include "LaserPoints.h"
#include "KNNFinder.h"

using namespace std;

void PrintUsage()
{
    printf("Usage: raycasting -traj_points <trajectory laserpoints>\n");
    printf("                  -laser_points <input laserpoints>\n");
    printf("                  -out_current_laserpoints <export points and belonging trajectory>\n");
}


int main(int argc, char *argv[]) {

    InlineArguments     *args = new InlineArguments(argc, argv);
    //void                raycasting(char* ascii_file, char *laserFile, double traj_resolution);
    void                PointWiseOcclusionTest(char* ascii_file, char *laserFile, double traj_resolution);
    vector <int>        occlusion_test(LaserPoint surface_point, LaserPoint source_point, LaserPoints lp);

    /// Check on required input files
    if (args->Contains("-usage") ||
        !args->Contains("-traj_points") ||
        !args->Contains("-laser_points"))
    {
        if (!args->Contains("-usage")) printf("Error: missing programme option.\n");
        PrintUsage();
        return EXIT_SUCCESS;
    }

    /// Call the main function (raycasting function)
    PointWiseOcclusionTest(args->String("-traj_points"), args->String("-laser_points"),
                     args->Contains("-traj_resolution"));


    //std::cout << "Press ENTER to continue...";
    //std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );

    return EXIT_SUCCESS;
}