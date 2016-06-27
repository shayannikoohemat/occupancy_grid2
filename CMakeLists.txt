cmake_minimum_required(VERSION 3.3)
project(occupancy_grid2)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


include_directories(D:/MappingLibrary/Mapping/Library/Photogrammetry/include)
include_directories(D:/MappingLibrary/Mapping/Library/Buildings/include)
include_directories(D:/MappingLibrary/Mapping/Library/LaserScan/include)
include_directories(D:/MappingLibrary/Mapping/Foreign/ANN/include/ANN)
link_directories(D:/MappingLibrary/Mapping/Library/lib64)
link_libraries(-lLaserScan -lPhotogrammetry -lBuildings -lEispack -lLinpack -lANN -lnewmat -lgfortran -g3 -llas)

set(SOURCE_FILES main.cpp traj2laserpoints.cpp laservoxel.cpp)
#set(SOURCE_FILES test.cpp)

add_executable(occupancy_grid2 ${SOURCE_FILES})

#target_link_libraries(occupancy_grid2 -lLaserScan -lPhotogrammetry -lBuildings -lEispack -lLinpack -lANN -lnewmat -lgfortran -g3 -llas)