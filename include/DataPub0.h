//
// Created by ly on 19-4-4.
//

#ifndef CATKIN_SLAM_DATAPUB_H
#define CATKIN_SLAM_DATAPUB_H

#include <ros/ros.h>

#include "System.h"

//#include <tf/transform_broadcaster.h>
//#include <nav_msgs/Path.h>
//#include <geometry_msgs/Pose.h>
//#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
//#include <image_transport/image_transport.h>
//#include <cv_briage/cv_briage.h>
//#include <sensor_msgs/image_encodings.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>

#include <boost/make_shared.hpp>

#include <mutex>
#include <thread>
#include <condition_variable>

namespace ORB_SLAM2
{

    class DataPub
    {
        typedef pcl::PointXYZRGBA PointT;
        typedef pcl::PointCloud<PointT> PointCloud;
    public:
        DataPub();//初始化，定义发布节点
        void run();
        void shutdown();
        void getPointCloud(PointCloud::Ptr curPointCloud );
    protected:
        ros::NodeHandle nh;
        ros::Publisher pcl_pub;
        sensor_msgs::PointCloud2 output;

        pcl::PointCloud<PointT> cloud;
        PointCloud::Ptr globalpc;

        size_t N = 0;
        size_t lastpcsize = 0;
        bool shutdownFlag = false;
        bool isPubData = false;

        condition_variable datapubUpdated;
        mutex shutdownMutex;
        mutex datapubMutex;
        mutex datapubUpdateMutex;

        shared_ptr<thread>  pubThread;

    };
}
#endif //CATKIN_SLAM_DATAPUB_H
