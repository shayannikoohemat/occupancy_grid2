//
// Created by NikoohematS on 7/1/2016.
//

#include <iostream>
#include <cstdlib>
#include <Matrix3.h>
#include <vector>
#include <Vector3D.h>
#include "LaserPoints.h"
#include "Laservoxel.h"
#include "LaserOctree.h"



int main(int argc, char *argv[]) {

//vector<int> occlusion_test(LaserPoint surface_point, LaserPoint source_point, LaserPoints lp) {



    /// read all laser points
    LaserPoints             lp;
    char                    *laserFile;
    laserFile = (char *) "D://test//points_crop_occlusion.laser";
    lp.Read(laserFile);

    ///construct octree
    DataBoundsLaser         bounds = lp.DeriveDataBounds(0);
    LaserOctree             lOctree;
    PointNumberList         pnlist;
    lOctree.Initialise(bounds.Bounds3D(), 0.1, 100);
    lOctree.Insert(lp);                             // Insert all laser points
    lOctree.Split(lp);                              // Split into children
    pnlist = lOctree.PointNumbers();                // ???

    int octree_cubes = lOctree.NumberOfCubes();
    int octree_depth = lOctree.Depth();

    printf ("octree # of cubes: %d \n", octree_cubes);  // extra info
    printf ("octree depth: %d \n", octree_depth);       // extra info


/*   LaserOctree           *octree;
    case 1: // Derive edges from octree
        octree = new LaserOctree();
    DeriveDataBounds(1);
    octree->Initialise(bounds.Bounds3D(),
                       parameters.OctreeBinOverlap(),
                       parameters.OctreeBinMaxNumberOfPoints());
    octree->Insert(*this);
          // Derive edges from octree
      edges = octree->NeighbourhoodEdges3D(*this,
                                           parameters.MaxDistanceInComponent());
      octree->Erase();
      nbh_edges = edges;
      return edges;
      break;*/

    /// read empty voxel grids
    LaserPoints             empty_voxel_points;
    char                    *laserFile_voxels;
    laserFile_voxels = (char *) "D://test//raycasting//empty_vox_centers.laser";
    empty_voxel_points.Read(laserFile_voxels);

    /// read matched_trajectory
    LaserPoints             matched_trajectory;
    char                    *laserFile_traj;
    laserFile_traj = (char *) "D://test//raycasting//matched_trajectories.laser";
    matched_trajectory.Read(laserFile_traj);

    Vector3D                v, direction;
    Line3D                  ray, threeDline;
    LineTopology            ray_line;
    LineTopologies          rays_line, all_rays_line;
    ObjectPoints            all_rays_points;
    LaserPoints::iterator   pt1, pt2;
    Position3D              pos1, pos2, origin;
    Positions3D             pos_pair;

    int                     line_nr = 0;
    double                  ray_length;

    for (pt1 = empty_voxel_points.begin(); pt1 != (empty_voxel_points.begin()+10); pt1++) {
        pos1 = Position3D(*pt1);  //on the surface // target

        for (pt2 = matched_trajectory.begin(); pt2 != (matched_trajectory.begin()+10); pt2++) {
            pos2 = Position3D(*pt2); // on the trajectory // origin
            ray = Line3D(Position3D(*pt1), Position3D(*pt2)); // line 3D between to points
            direction = ray.Direction();   // direction of the ray
            origin = pos2;                 // origin of the ray

            pos_pair.push_back(pos1);
            pos_pair.push_back(pos2);

            ObjectPoints objpoints_temp(pos_pair,0);
            ray_line = LineTopology(line_nr++, 0, objpoints_temp[0],objpoints_temp[1]);

            //ray_length = pos1.Distance(pos2);  // use later as ray length

            rays_line.push_back(ray_line);

            if (!all_rays_line.empty())
                rays_line.ReNumber(objpoints_temp, (all_rays_points.end()-1)->Number()+1, (all_rays_points.end()-1)->Number()+1);
            ///Store all the lines and their points
            all_rays_line.insert(all_rays_line.end(), rays_line.begin(), rays_line.end());
            all_rays_points.insert(all_rays_points.end(), objpoints_temp.begin(), objpoints_temp.end());


            objpoints_temp.Erase();
            rays_line.Erase();
            pos_pair.clear();
        }
    }
    all_rays_line.Write("D://test//raycasting//rays.top");
    all_rays_points.Write("D://test//raycasting//rays.objpts");

    return 0;
}

/// Raycasting implementation from Revelles et al. 2000 paper
using namespace std;

unsigned char a; // because an unsigned char is 8 bits

int first_node(double tx0, double ty0, double tz0, double txm, double tym, double tzm){
    unsigned char answer = 0;   // initialize to 00000000
// select the entry plane and set bits
    if(tx0 > ty0){
        if(tx0 > tz0){ // PLANE YZ
            if(tym < tx0) answer|=2;    // set bit at position 1
            if(tzm < tx0) answer|=1;    // set bit at position 0
            return (int) answer;
        }
    }
    else {
        if(ty0 > tz0){ // PLANE XZ
            if(txm < ty0) answer|=4;    // set bit at position 2
            if(tzm < ty0) answer|=1;    // set bit at position 0
            return (int) answer;
        }
    }
// PLANE XY
    if(txm < tz0) answer|=4;    // set bit at position 2
    if(tym < tz0) answer|=2;    // set bit at position 1
    return (int) answer;
}

int new_node(double txm, int x, double tym, int y, double tzm, int z){
    if(txm < tym){
        if(txm < tzm){return x;}  // YZ plane
    }
    else{
        if(tym < tzm){return y;} // XZ plane
    }
    return z; // XY plane;
}

void proc_subtree (double tx0, double ty0, double tz0, double tx1, double ty1, double tz1, Node* node){
    float txm, tym, tzm;
    int currNode;

    if(tx1 < 0 || ty1 < 0 || tz1 < 0) return;
    if(node->terminal){
        cout << "Reached leaf node " << node->debug_ID << endl;
        return;
    }
    else{ cout << "Reached node " << node->debug_ID << endl;}

    txm = 0.5*(tx0 + tx1);
    tym = 0.5*(ty0 + ty1);
    tzm = 0.5*(tz0 + tz1);

    currNode = first_node(tx0,ty0,tz0,txm,tym,tzm);
    do{
        switch (currNode)
        {
            case 0: {
                proc_subtree(tx0,ty0,tz0,txm,tym,tzm,node->children[a]);
                currNode = new_node(txm,4,tym,2,tzm,1);
                break;}
            case 1: {
                proc_subtree(tx0,ty0,tzm,txm,tym,tz1,node->children[1^a]);
                currNode = new_node(txm,5,tym,3,tz1,8);
                break;}
            case 2: {
                proc_subtree(tx0,tym,tz0,txm,ty1,tzm,node->children[2^a]);
                currNode = new_node(txm,6,ty1,8,tzm,3);
                break;}
            case 3: {
                proc_subtree(tx0,tym,tzm,txm,ty1,tz1,node->children[3^a]);
                currNode = new_node(txm,7,ty1,8,tz1,8);
                break;}
            case 4: {
                proc_subtree(txm,ty0,tz0,tx1,tym,tzm,node->children[4^a]);
                currNode = new_node(tx1,8,tym,6,tzm,5);
                break;}
            case 5: {
                proc_subtree(txm,ty0,tzm,tx1,tym,tz1,node->children[5^a]);
                currNode = new_node(tx1,8,tym,7,tz1,8);
                break;}
            case 6: {
                proc_subtree(txm,tym,tz0,tx1,ty1,tzm,node->children[6^a]);
                currNode = new_node(tx1,8,ty1,8,tzm,7);
                break;}
            case 7: {
                proc_subtree(txm,tym,tzm,tx1,ty1,tz1,node->children[7^a]);
                currNode = 8;
                break;}
        }
    } while (currNode<8);
}

void ray_octree_traversal(Octree* octree, Ray ray){
    a = 0;

// fixes for rays with negative direction
    if(ray.direction[0] < 0){
        ray.origin[0] = octree->size[0] - ray.origin[0];
        ray.direction[0] = - ray.direction[0];
        a |= 4 ; //bitwise OR (latest bits are XYZ)
    }
    if(ray.direction[1] < 0){
        ray.origin[1] = octree->size[1] - ray.origin[1];
        ray.direction[1] = - ray.direction[1];
        a |= 2 ;
    }
    if(ray.direction[2] < 0){
        ray.origin[2] = octree->size[2] - ray.origin[2];
        ray.direction[2] = - ray.direction[2];
        a |= 1 ;
    }

    double divx = 1 / ray.direction[0]; // IEEE stability fix
    double divy = 1 / ray.direction[1];
    double divz = 1 / ray.direction[2];

    double tx0 = (octree->min[0] - ray.origin[0]) * divx;
    double tx1 = (octree->max[0] - ray.origin[0]) * divx;
    double ty0 = (octree->min[1] - ray.origin[1]) * divy;
    double ty1 = (octree->max[1] - ray.origin[1]) * divy;
    double tz0 = (octree->min[2] - ray.origin[2]) * divz;
    double tz1 = (octree->max[2] - ray.origin[2]) * divz;

    if( max(max(tx0,ty0),tz0) < min(min(tx1,ty1),tz1) ){
        proc_subtree(tx0,ty0,tz0,tx1,ty1,tz1,octree->root);
    }
}

