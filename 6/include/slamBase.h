//#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <iostream>
using namespace std;

#include <Eigen/Core>  
#include <Eigen/Geometry> 

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

//using namespace cv;

struct CAMERA_INTRINSIC_PARAMETERS
{
    double factor;
    double cx;
    double cy;
    double fx;
    double fy;
};

struct FRAME
{
	cv::Mat rgb, depth;//该帧对应的彩色图与深度图
    cv::Mat desp;      //特征描述子
    vector<cv::KeyPoint> kp; //关键点
};

//PnP结果
struct RESULT_OF_PNP
{ 
    cv::Mat rvec, tvec;
    int inliers;
};

// 参数读取类
class ParameterReader
{
public:
    ParameterReader( string filename="./parameters.txt" )
    {        
        ifstream fin( filename.c_str() );

        //if (!fin)
        if(fin.peek() == EOF)
        {
            cerr<<"parameter file does not exist."<<endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                // 以‘＃’开头的是注释
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }
public:
    map<string, string> data;
};

PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera );
cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera );

//同时提取关键点与特征描述子
void computeKeyPointsAndDesp(FRAME& frame, string detector );

//估计两帧之间的运动
RESULT_OF_PNP estimateMotion(FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera);

// cvMat2Eigen
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec );

// joinPointCloud 
PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera ) ;

inline static CAMERA_INTRINSIC_PARAMETERS getDefaultCamera()
{
    ParameterReader pd;
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.fx = atof( pd.getData( "camera.fx" ).c_str());
    camera.fy = atof( pd.getData( "camera.fy" ).c_str());
    camera.cx = atof( pd.getData( "camera.cx" ).c_str());
    camera.cy = atof( pd.getData( "camera.cy" ).c_str());
    camera.factor = atof( pd.getData( "camera.factor" ).c_str() );
    return camera;
}
