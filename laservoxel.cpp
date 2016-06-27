
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


#include "laservoxel.h"

LaserVoxel::LaserVoxel(LaserPoints ls, double vox_l)
{
 vox_length=vox_l;
 bool verbose=1;

 //get the databounds from the ls
 DataBoundsLaser db=ls.DeriveDataBounds(0);
 min_X=db.Minimum().GetX();
 min_Y=db.Minimum().GetY();
 min_Z=db.Minimum().GetZ();

//get the databounds from the ls TO visualize
LineTopology               rect_top;
ObjectPoints               rect_objpts;
LineTopologies             bounding_box;
ls.DeriveTIN();
db=ls.DeriveDataBounds(0);
ls.EnclosingRectangle(0.1, rect_objpts, rect_top);
rect_objpts.Write("D:\\test\\bounding_box.objpts");
bounding_box.insert(bounding_box.end(),rect_top);
bounding_box.Write("D:\\test\\bounding_box.top", false);
 
 //length of vector componetnts: compute index for max vals - +1;
 vox_num_X=index_from_realX(db.Maximum().GetX())+1;
 vox_num_Y=index_from_realY(db.Maximum().GetY())+1;
 vox_num_Z=index_from_realZ(db.Maximum().GetZ())+1;
 if (verbose)
 {
 printf("Number of Laserpoints:%d\n",ls.size()); 
 printf("length of voxel:%.2f\n",vox_length);
 printf("min_X=%.2f, min_Y=%.2f, min_Z=%.2f\n",min_X,min_Y,min_Z);
 printf("num of voxels in X:%d, Y:%d, Z:%d: Total:%d\n",vox_num_X,vox_num_Y,vox_num_Z,vox_num_X*vox_num_Y*vox_num_Z);
 
 printf("...allocating memory...wait\n");
 }
 //allocate memory
 Vox.resize(vox_num_X);
 for (uint i=0;i<vox_num_X;i++)
 {
   Vox[i].resize(vox_num_Y);
   
   for (uint j=0; j<vox_num_Y;j++)
   {
     Vox[i][j].resize(vox_num_Z);
     
     for (uint k=0;k<vox_num_Z;k++)
       Vox[i][j][k]=NULL;
   }
   
 }
 
 if ( verbose) printf("...putting data into structure...wait\n");
 //put the data into the voxel
 LaserPoints::iterator point;
    for (point=ls.begin(); point!=ls.end(); point++)
	  {
	    LaserPoint lp=point->LaserPointRef();
	    double X=lp.GetX();
	    double Y=lp.GetY();
	    double Z=lp.GetZ();
	    
	    uint i=index_from_realX(X);
	    uint j=index_from_realY(Y);
	    uint k=index_from_realZ(Z);
	    
	    //printf("in reading: i=%d, j=%d, k=%d\n",i,j,k);
	    if (Vox[i][j][k]==NULL) Vox[i][j][k]=new LaserPoints;
	    
	    Vox[i][j][k]->push_back(lp);
	    
	  }
if (verbose) printf("...done!\n");
} //Constructor

LaserVoxel::~LaserVoxel()
{
 for (uint i=0;i<vox_num_X;i++)
   for (uint j=0; j<vox_num_Y;j++)
     for (uint k=0;k<vox_num_Z;k++)
     {
       if (Vox[i][j][k]!=NULL) delete(Vox[i][j][k]);
      } 
}

void LaserVoxel::statistics()
{
::map<uint, uint > statmap; //first: number of points per voxel, second: occurence
  for (uint i=0;i<vox_num_X;i++)
   for (uint j=0; j<vox_num_Y;j++)
     for (uint k=0;k<vox_num_Z;k++)
     {
       if (Vox[i][j][k]!=NULL) 
       {
	 //printf("in cell %d %d %d: %d points\n",i,j,k,Vox[i][j][k]->LaserPointsReference().size());
	uint size=Vox[i][j][k]->LaserPointsReference().size();
	statmap[size]++;
       }
     }
 
printf("\nStatistics:\n"); 
uint total_voxels=0;
for (::map<uint, uint>::iterator it=statmap.begin();it!=statmap.end();it++)
{
    printf("number of points/voxel: %d, occurence: %d\n",it->first,it->second);
    total_voxels+=it->second;
}
printf("\nTotal number of voxels:%d\n",total_voxels);
printf("\n\n");
}

void LaserVoxel::filter_size(uint min,bool verbose)
{
  if (verbose) printf("Voxel fiter: min number of points per voxel required: %d\n",min);
 for (uint i=0;i<vox_num_X;i++)
   for (uint j=0;j<vox_num_Y;j++)
     for (uint k=0;k<vox_num_Z;k++)
     {
       if (Vox[i][j][k]!=NULL)
	 if (Vox[i][j][k]->LaserPointsReference().size() < min) 
	 {//delete(Vox[i][j][k]);
	  Vox[i][j][k]=NULL;
	 }
      }  
   if (verbose) printf("...done\n");
}

void LaserVoxel::filter_neighbour(uint min, bool verbose)
{
  if (verbose) printf("Voxel fiter: min number of points in neighborhood required: %d\n",min);
  for (uint i=0;i<vox_num_X;i++)
   for (uint j=0; j<vox_num_Y;j++)
     for (uint k=0;k<vox_num_Z;k++)
     {
       if (Vox[i][j][k]!=NULL)
       {
	 uint found_p=0;
	 uint zero=0;
	
	   for (uint a=max(i-1,zero);a<=i+1;a++) //the indices may not be <0
	   {
	      if (a>=vox_num_X) continue;
	      
	      for (uint b=max(j-1,zero);b<=j+1;b++)
	      {
		if (b>=vox_num_Y) continue;
		  
		for (uint c=max(k-1,zero);c<=k+1;c++)
		  {if (c>=vox_num_Z) continue;
		      if (a==i && b==j && c==k) continue; //the point itself is not counted
			if (Vox[a][b][c]!=NULL)
			  found_p+=Vox[a][b][c]->LaserPointsReference().size(); 			
		    }//c
	      }//b	   
	   }//a
	   if (found_p<min) Vox[i][j][k]=NULL;
       }
      }
  if (verbose) printf("...done\n");
}

LaserPoints LaserVoxel::export_all()
{
 LaserPoints l;
  for (uint i=0;i<vox_num_X;i++)
   for (uint j=0; j<vox_num_Y;j++)
     for (uint k=0;k<vox_num_Z;k++)
     if (Vox[i][j][k]!=NULL) 
      {
       //l=l+Vox[i][j][k]->LaserPointsReference(); //+ operator too expensive
	  LaserPoints::iterator point;
	  LaserPoints tmpl=Vox[i][j][k]->LaserPointsReference();
	  for (point=tmpl.begin(); point!=tmpl.end(); point++)
	  {
	    LaserPoint lp=point->LaserPointRef();
	    l.push_back(lp);
	  }
      }
  
  return l;
}

LaserPoints LaserVoxel::export_vox_centres()
{
LaserPoints l;
  for (uint i=0;i<vox_num_X;i++)
   for (uint j=0; j<vox_num_Y;j++)
     for (uint k=0;k<vox_num_Z;k++)
     if (Vox[i][j][k]!=NULL) 
      {
	double X=min_X+(double) ((double) (i+0.5)* vox_length);
        double Y=min_Y+(double) ((double) (j+0.5)* vox_length);
        double Z=min_Z+(double) ((double) (k+0.5)* vox_length);
	
         //also approximate the color...
	  int rsum=0, gsum=0, bsum=0, num=0;
	  LaserPoints::iterator point;
	  LaserPoints tmpl=Vox[i][j][k]->LaserPointsReference();
	  for (point=tmpl.begin(); point!=tmpl.end(); point++)
	  {
	    LaserPoint lp=point->LaserPointRef();
	    rsum+=lp.Red();
            gsum+=lp.Green();
	    bsum+=lp.Blue();
	    num++;
	  }
       
	 LaserPoint lp(X,Y,Z);
	 lp.SetColour((int) rsum/num,(int) gsum/num, (int) bsum/num);

	 l.push_back(lp);
	  
      }
         // to export empty voxel-centers (empty = voxel without point)
     else
     {
         LaserPoint lp (min_X+i*vox_length-vox_length/2,min_Y+j*vox_length-vox_length/2,min_Z+k*vox_length-vox_length/2);
         lp.SetAttribute(LabelTag,100);

         l.push_back(lp);
     }
  
  return l;

}
