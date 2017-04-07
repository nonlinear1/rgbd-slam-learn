#增加pcl库依赖
#FIND_PACKAGE( PCL REQUIRED COMPONENTS common io )
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04

#增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )

#添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

# 添加g2o的依赖
#因为g2o不是常用库，要添加它的findg2o.camke文件
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )
FIND_PACKAGE( G2O REQUIRED )
#CSparse
FIND_PACKAGE( CSparse REQUIRED )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )

#TARGET_LINK_LIBRARIES( generate_pointcloud hello_share)

#ADD_EXECUTABLE( main main.cpp )

#封装库文件
#ADD_LIBRARY( hello libhello.cpp)
#封装库文件
#ADD_LIBRARY( hello_share SHARED libhello.cpp)

#封装库文件
ADD_LIBRARY( slambase_share SHARED slamBase.cpp)

#ADD_EXECUTABLE( 111 111.cpp )
#TARGET_LINK_LIBRARIES( 111 hello_share slambase_share ${OpenCV_LIBS}
#		${PCL_LIBRARIES} )

ADD_EXECUTABLE( generate_pointcloud generatePointCloud.cpp )
TARGET_LINK_LIBRARIES( generate_pointcloud slambase_share ${OpenCV_LIBS}
		${PCL_LIBRARIES} )

ADD_EXECUTABLE( detectFeatures detectFeatures.cpp )
TARGET_LINK_LIBRARIES( detectFeatures slambase_share ${OpenCV_LIBS}
		${PCL_LIBRARIES} )

ADD_EXECUTABLE( jointPointCloud jointPointCloud.cpp )
TARGET_LINK_LIBRARIES( jointPointCloud slambase_share ${OpenCV_LIBS}
		${PCL_LIBRARIES} )

ADD_EXECUTABLE( visualOdometry visualOdometry.cpp )
TARGET_LINK_LIBRARIES( visualOdometry slambase_share ${OpenCV_LIBS}
		${PCL_LIBRARIES} )

ADD_EXECUTABLE( slamEnd slamEnd.cpp )
TARGET_LINK_LIBRARIES( slamEnd 
		slambase_share 
		${OpenCV_LIBS}
		${PCL_LIBRARIES}
		g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension  ${CSPARSE_LIBRARY}	
 )

