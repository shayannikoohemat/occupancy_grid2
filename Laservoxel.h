
/*
    Copyright 2010 University of Twente and Delft University of Technology
 
       This file is part of the Mapping libraries and tools, developed
  for research, education and projects in photogrammetry and laser scanning.

  The Mapping libraries and tools are free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation, either version 3 of the License,
                   or (at your option) any later version.

 The Mapping libraries and tools are distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
                GNU General Public License for more details.

      You should have received a copy of the GNU General Public License
          along with the Mapping libraries and tools.  If not, see
                      <http://www.gnu.org/licenses/>.

----------------------------------------------------------------------------*/


#ifndef LaserVoxel_H
#define LaserVoxel_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "LaserPoints.h"
#include <vector>
#include <map>


#ifndef uint
typedef unsigned int uint;
#endif


class LaserVoxel
{
  
  private:
    vector< vector < vector < LaserPoints* > > > Vox;
    ///edge length of a voxel
    double vox_length; 
    ///number of voxels in XYZ
    uint vox_num_X, vox_num_Y, vox_num_Z;
    ///min value in real X Y Z (bounding box)
    double min_X, min_Y, min_Z;
    ///given a real object coordinate the indices for the Voxel field are computed
    uint index_from_real(double real, double min) {return (uint) floor((real-min)/vox_length);}
    uint index_from_realX(double real) {return (index_from_real(real,min_X));}
    uint index_from_realY(double real) {return (index_from_real(real,min_Y));}
    uint index_from_realZ(double real) {return (index_from_real(real,min_Z));}
    
 public:
   LaserVoxel(LaserPoints ls,double vox_l);
   ~LaserVoxel();
  
   void statistics();
   
   ///Filter: min. number of points in a voxel
   void filter_size(uint min=2,bool verbose=1);
   ///Filter according to neighb.hood: in the 26-neighb.hood of a voxel a min. number of points is required, otherwise it is deleted
   void filter_neighbour(uint min=1,bool verbose=1);
   LaserPoints export_all();
   LaserPoints export_vox_centres(); ///only exports the centres of voxels (where laserpoints are)

};



#endif
