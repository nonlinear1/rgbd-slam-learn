FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04

#添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

ADD_EXECUTABLE( main main.cpp )
ADD_EXECUTABLE(pcl_pass_filter pcl_pass_filter.cpp)
TARGET_LINK_LIBRARIES( pcl_pass_filter ${PCL_LIBRARIES} )

ADD_EXECUTABLE(pcl_voxel_filter pcl_voxel_filter.cpp)
TARGET_LINK_LIBRARIES( pcl_voxel_filter ${PCL_LIBRARIES} )

