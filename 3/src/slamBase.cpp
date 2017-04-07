#include <slamBase.h>

cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera )
{ 

            //定义一个3d点
            cv::Point3f p;

            //计算这点的空间坐标
            p.z = double(point.z)/camera.factor;
            p.x = ( point.x-camera.cx )*p.z / camera.fx;
            p.y = ( point.y-camera.cy )*p.z / camera.fy;

            return p;
}

// image2PonitCloud 将rgb图转换为点云
PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    PointCloud::Ptr cloud ( new PointCloud );
    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标
            p.z = double(d) / camera.factor;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;
            
            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout<<"point cloud size = "<<cloud->points.size()<<endl;
    cloud->is_dense = false;
    return cloud;
}

void computeKeyPointsAndDesp(FRAME& frame, string detector )
{
    //声明特征提取器与描述子提取器
    cv::Ptr<cv::FeatureDetector> detector1;

    cout << detector.c_str() << endl;
    //构建提取器
    if ("ORB" == detector)
    {
        detector1 = cv::ORB::create();
    }
    if (!detector1)
    {
        cerr<<"Unknown detector or discriptor type !"<<detector<<endl;
        return;
    }

    detector1->detect(frame.rgb, frame.kp); //提取关键点
    detector1->compute( frame.rgb, frame.kp, frame.desp );

}

RESULT_OF_PNP estimateMotion(FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera)
{
    static ParameterReader pd;

    //匹配描述子
    vector<cv::DMatch> matches;
    cv::BFMatcher matcher;;
    matcher.match( frame1.desp, frame2.desp, matches );
    cout<<"find total "<<matches.size()<<" matches."<<endl;

    //筛选匹配，把距离太大的去掉
    //这里使用的准则是去掉大于四倍最小距离的匹配
    vector<cv::DMatch> goodMatches;
    double minDis = 9999;
    for(size_t i=0; i<matches.size();i++)
    {
        if(matches[i].distance < minDis)
            minDis = matches[i].distance;
    }
    for(size_t i=0;i<matches.size();i++)
    {
        if(matches[i].distance  < 4*minDis)
            goodMatches.push_back(matches[i]);
    }

    //计算图像间的运动关系
    //关键函数：cv::solvePnPRansac()
    //为调用此函数准备必要的参数

    //第一个帧的三维点
    vector<cv::Point3f> pts_obj;
    //第二个帧的图像点
    vector<cv::Point2f> pts_img;

    for(size_t i=0;i<goodMatches.size();i++)
    {
        //query是第一个，train是第二个
        cv::Point2f p = frame1.kp[goodMatches[i].queryIdx].pt;
        //获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        ushort d = frame1.depth.ptr<ushort>(int(p.y))[int(p.x)];
        if( d ==0 )
            continue;
        pts_img.push_back(cv::Point2f(frame2.kp[goodMatches[i].trainIdx].pt));

        //将(u,v,d)转化成(x,y,z)
        cv::Point3f pt (p.x, p.y ,d);
        cv::Point3f pd = point2dTo3d(pt, camera);
        pts_obj.push_back(pd);
    }

    double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
    };

    //构建相机矩阵
    cv::Mat cameraMatrix(3, 3, CV_64F, camera_matrix_data);
    cv::Mat rvec, tvec, inliers;
    //求解pnp
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 8.0, 0.99, inliers);

    cout << "inliers: " << inliers.rows << endl;
    cout << "R=" << rvec << endl;
    cout << "t=" << tvec << endl;

    RESULT_OF_PNP pnp_result;
    pnp_result.rvec = rvec;
    pnp_result.tvec = tvec;
    pnp_result.inliers = inliers.rows;

    // //画出inliers匹配
    // vector<cv::DMatch> matchesShow;
    // for(size_t i=0;i<inliers.rows;i++)
    // {
    //     matchesShow.push_back(goodMatches[inliers.ptr<int>(i)[0]]);
    // }
    // cv::drawMatches(rgb1, kp1, rgb2, kp2, matchesShow, imgMatches);
    // cv::imshow("inlier matches", imgMatches);
    // cv::imwrite("./data/inliers.png", imgMatches);
    // cv::waitKey(0);


    return pnp_result;
}


