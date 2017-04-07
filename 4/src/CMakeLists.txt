#增加pcl库依赖
#FIND_PACKAGE( PCL REQUIRED COMPONENTS common io )
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04

#增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )

#添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )


ADD_EXECUTABLE( generate_pointcloud generatePointCloud.cpp )
TARGET_LINK_LIBRARIES( generate_pointcloud hello_share slambase_share ${OpenCV_LIBS}
		${PCL_LIBRARIES} )

#TARGET_LINK_LIBRARIES( generate_pointcloud hello_share)
#TARGET_LINK_LIBRARIES( generate_pointcloud slambase_share)

ADD_EXECUTABLE( main main.cpp )

#封装库文件
ADD_LIBRARY( slambase_share SHARED slamBase.cpp)

ADD_EXECUTABLE( 111 111.cpp )
TARGET_LINK_LIBRARIES( 111 hello_share slambase_share ${OpenCV_LIBS}
		${PCL_LIBRARIES} )

ADD_EXECUTABLE( detectFeatures detectFeatures.cpp )
TARGET_LINK_LIBRARIES( detectFeatures slambase_share ${OpenCV_LIBS}
		${PCL_LIBRARIES} )

ADD_EXECUTABLE( jointPointCloud jointPointCloud.cpp )
TARGET_LINK_LIBRARIES( jointPointCloud slambase_share ${OpenCV_LIBS}
		${PCL_LIBRARIES} )
