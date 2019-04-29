//
// Created by ly on 19-4-4.
//

#ifndef CATKIN_SLAM_DATAPUB_H
#define CATKIN_SLAM_DATAPUB_H

#include <ros/ros.h>

#include "System.h"
#include "Converter.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
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
        DataPub( const string &strSettingsFile );//初始化，定义发布节点
        void PubPointCloud();
	    void PubCamPose();
        void shutdown();
        void getPointCloud(PointCloud::Ptr curPointCloud );
        void getCameraPose(const cv::Mat& Tcw);
	    void GetCurROSCamMartrix(geometry_msgs::PoseStamped& campose2ground);
        Eigen::Isometry3d getTransformation();
	//void GetCurROSTrajectories(nav_msgs::Path& campath);
    protected:
        ros::NodeHandle nh;
        ros::Publisher pcl_pub;
	    ros::Publisher CamPose_pub;
	    ros::Publisher CamPath_pub;
        sensor_msgs::PointCloud2 output;
	    tf::TransformBroadcaster Send_Cam2Ground;
	    tf::TransformBroadcaster Send_Robot2Ground;
	    tf::TransformListener listener;

	bool isGetNewPose = false;

        pcl::PointCloud<PointT> cloud;
        PointCloud::Ptr globalpc;
	cv::Mat mCameraPose;

        size_t N = 0;
        size_t lastpcsize = 0;
        bool shutdownFlag = false;
        bool isPubData = false;
	    double mT;

        condition_variable datapubUpdated;
        mutex shutdownMutex;
        mutex datapubMutex;
        mutex datapubUpdateMutex;
        condition_variable cameraPoseUpdated;
	    mutex cameraPoseMutex;
	    mutex shutdown_Mutex;
	    mutex datapubUpdate_Mutex;

        shared_ptr<thread>  pubPointCloudThread;
	    shared_ptr<thread>  pubCamPoseThread;

	    Eigen::Matrix4f mTransCam2Ground;
	    Eigen::Matrix4f mCam2Ground;
	    Eigen::Matrix4f mRob2Ground;
    };
}
#endif //CATKIN_SLAM_DATAPUB_H
