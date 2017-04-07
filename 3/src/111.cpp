#include <iostream>
#include <string>

#include <libhello.h>
#include <slamBase.h>

using namespace std;
using namespace cv;

int main( int argc, char** argv )
{ 
    //相机内参
    const double camera_factor = 1000;
    const double camera_cx = 325.5;
    const double camera_cy = 253.5;
    const double camera_fx = 518.0;
    const double camera_fy = 519.0;
    struct CAMERA_INTRINSIC_PARAMETERS camera = {1000, 325.5, 253.5, 518.0, 519.0};

    // //读取rgb和深度图像，并转化为点云
    // Mat rgb, depth;
    // print_hello();
    // //rgb图像是8UC3的彩色图像
    // //depth是16UC1的单通道图像，注意flags设置-1,表示原始数据不做任何修改
    // rgb = imread( "./data/rgb.png" );
    // depth = imread( "./data/depth.png", -1 );

    // PointCloud::Ptr cloud = image2PointCloud(rgb, depth, camera);
    // pcl::io::savePCDFile( "./111.pcd", *cloud );

    // //清除数据并退出
    // cloud->points.clear();
    // cout<< "Point cloud saved."<< endl;

    FRAME frame1, frame2;
    //声明并从data文件夹中读取两个rgb与深度图
    frame1.rgb = cv::imread( "./data/rgb1.png" );
    frame2.rgb = cv::imread( "./data/rgb2.png" );
    frame1.depth = cv::imread( "./data/depth1.png" );
    frame2.depth = cv::imread( "./data/depth2.png" );

    string detector = "ORB";
    computeKeyPointsAndDesp( frame1, detector );
    computeKeyPointsAndDesp( frame2, detector );
    cout << "Key points of two images:" << frame1.kp.size() << endl;

    estimateMotion(frame1, frame2, camera);

    return 0; 
}

