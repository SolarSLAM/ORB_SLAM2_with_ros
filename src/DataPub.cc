//
// Created by ly on 19-4-4.
//

#include "DataPub.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/core/eigen.hpp>

#include <mutex>

namespace ORB_SLAM2
{
        DataPub::DataPub( const string &strSettingsFile )
        {
	    cv::FileStorage fSettings(strSettingsFile,cv::FileStorage::READ);
	    float fps = fSettings["Camera.fps"];
    		if(fps<1)
     		   fps=30;
  	    mT = 1e3/fps;

	    mTransCam2Ground.setIdentity();// Set to Identity to make bottom row of Matrix 0,0,0,1
    	    mTransCam2Ground(0,0) = fSettings["Transformation.R00"];
            mTransCam2Ground(0,1) = fSettings["Transformation.R01"];
            mTransCam2Ground(0,2) = fSettings["Transformation.R02"];
            mTransCam2Ground(0,3) = fSettings["Transformation.tx"];
            mTransCam2Ground(1,0) = fSettings["Transformation.R10"];
            mTransCam2Ground(1,1) = fSettings["Transformation.R11"];
            mTransCam2Ground(1,2) = fSettings["Transformation.R12"];
            mTransCam2Ground(1,3) = fSettings["Transformation.ty"];
            mTransCam2Ground(2,0) = fSettings["Transformation.R20"];
            mTransCam2Ground(2,1) = fSettings["Transformation.R21"];
            mTransCam2Ground(2,2) = fSettings["Transformation.R22"];
            mTransCam2Ground(2,3) = fSettings["Transformation.tz"];

            globalpc = boost::make_shared< PointCloud >();
            pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_output",1);
	        CamPose_pub = nh.advertise<geometry_msgs::PoseStamped>("camera_pose",1);
	        CamPath_pub = nh.advertise<nav_msgs::Path>("orb_point",1);
            pubPointCloudThread = make_shared<thread>(bind(&DataPub::PubPointCloud,this));
	        pubCamPoseThread = make_shared<thread>(bind(&DataPub::PubCamPose,this));
        }
        void DataPub::shutdown()
        {
            {
                unique_lock<mutex> lck(shutdownMutex);
                shutdownFlag = true;
                datapubUpdated.notify_one();
            }
            pubPointCloudThread->join();
            pubCamPoseThread->join();
        }

        void DataPub::getPointCloud(PointCloud::Ptr curPointCloud )
        {
            unique_lock<mutex> lck(datapubMutex);
            if(!isPubData)
                *globalpc += *curPointCloud;
            datapubUpdated.notify_one();
        }

        Eigen::Isometry3d DataPub::getTransformation()
        {
            Eigen::Isometry3d T1 = ORB_SLAM2::Converter::toSE3Quat(ORB_SLAM2::Converter::toCvMat(mTransCam2Ground));
            return T1;
        }

        void DataPub::PubPointCloud()
        {
            ros::Rate loop_rate(10);
            while(1)
            {

                {
                    unique_lock<mutex> lck_shutdown(shutdownMutex);
                    if(shutdownFlag)
                        break;
                }
                {
                    unique_lock<mutex> lck_datapubUpdated( datapubUpdateMutex );
                    datapubUpdated.wait( lck_datapubUpdated );
                }
                {
                    unique_lock<mutex> lck(datapubMutex);
                    N = globalpc->size();
                }
                if(N > lastpcsize)
                {
                    isPubData = true;
                    pcl::toROSMsg(*globalpc,output);
                    output.header.frame_id = "map";
                    output.header.stamp = ros::Time::now();
                    pcl_pub.publish(output);
                    ros::spinOnce();
                    loop_rate.sleep();
                    cout<<"完成"<<globalpc->size()<<"发送"<<endl;
                    isPubData = false;
                    lastpcsize = N;
                }

            }
        }

        void DataPub::getCameraPose(const cv::Mat& Tcw)
        {
                unique_lock<mutex> lck_pose(cameraPoseMutex);
                isGetNewPose = true;
                mCameraPose = Tcw.clone();
        }

        void DataPub::PubCamPose()
        {
            geometry_msgs::PoseStamped camPose2Ground;
            nav_msgs::Path CamPath;

            while(1)
            {
                bool isPubTF = false;
                    isPubTF = isGetNewPose;
                    GetCurROSCamMartrix(camPose2Ground);
                    cameraPoseUpdated.notify_one();
                if(isPubTF)
                {
                    //GetCurROSTrajectories(CamPath);
                    CamPose_pub.publish(camPose2Ground);
                    //CamPath_pub.publish(CamPath);

                    Eigen::Matrix4f T = mTransCam2Ground;
                    mRob2Ground = mCam2Ground * T.inverse();

                    Eigen::Matrix3f Rwr=mRob2Ground.block<3,3>(0,0);
                    Eigen::Quaternionf qwr(Rwr);
                    float r2w_qx=qwr.x();
                    float r2w_qy=qwr.y();
                    float r2w_qz=qwr.z();
                    float r2w_qw=qwr.w();
                    float r2w_x=mRob2Ground(0,3);
                    float r2w_y=mRob2Ground(1,3);
                    float r2w_z=mRob2Ground(2,3);

                    tf::Transform transform;
                    transform.setOrigin(tf::Vector3(r2w_x,r2w_y,r2w_z));
                    transform.setRotation(tf::Quaternion(r2w_qx,r2w_qy,r2w_qz,r2w_qw));
                    Send_Cam2Ground.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"map","zed_center"));
                    //cout<<"-----发布TF树----"<<endl;
                }
            }
        }
        void DataPub::GetCurROSCamMartrix(geometry_msgs::PoseStamped& campose2ground)
        {
            if(!mCameraPose.empty())
            {
                Eigen::Matrix4f cam_pose21stcam;
                Eigen::Matrix4f cam_pose2ground;
                Eigen::Matrix4f rob_pose2ground;
                {
                    unique_lock<mutex> lck_pose(cameraPoseMutex);
                    cv::cv2eigen(mCameraPose.inv(),cam_pose21stcam);
                    cam_pose2ground = mTransCam2Ground * cam_pose21stcam;
                    mCam2Ground = cam_pose2ground;
                }
                campose2ground.pose.position.x = cam_pose2ground(0,3);
                campose2ground.pose.position.y = cam_pose2ground(1,3);
                campose2ground.pose.position.z = cam_pose2ground(2,3);
                Eigen::Matrix3f Rwc = cam_pose2ground.block<3,3>(0,0);

                Eigen::Quaternionf q(Rwc);
                campose2ground.pose.orientation.x = q.x();
                campose2ground.pose.orientation.y = q.y();
                campose2ground.pose.orientation.z = q.z();
                campose2ground.pose.orientation.w = q.w();
                campose2ground.header.frame_id="map";
                campose2ground.header.stamp = ros::Time::now();
                isGetNewPose = false ;
            }
        }
    #if 0
        void DataPub::GetCurROSTrajectories(nav_msgs::Path& campath)
        {
            if(!mCameraPose.empty())
            {
                nav_msgs::Path cam_path_temp;
                geometry_msgs::PoseStamped cam_pose;
                vector<cv::Mat> curTrajectories;
                mpSystem->GetCurTrajectories(curTrajectories);
                Eigen::Matrix4f cam_pose_tmp;
                for(auto mt:curTrajectories)
                {
                    cv2eigen(mt,cam_pose_tmp);
                    Eigen::Matrix4f cam_pose2ground = mTransCam2Ground * cam_pose_temp;
                    cam_pose.pose.position.x=cam_pose2ground(0,3);
                        cam_pose.pose.position.y=cam_pose2ground(1,3);
                        cam_pose.pose.position.z=cam_pose2ground(2,3);
                        Eigen::Matrix3f Rwc=cam_pose2ground.block<3,3>(0,0);
                        Eigen::Quaternionf q(Rwc);
                        cam_pose.pose.orientation.x=q.x();
                        cam_pose.pose.orientation.y=q.y();
                        cam_pose.pose.orientation.z=q.z();
                        cam_pose.pose.orientation.w=q.w();
                        cam_path_temp.poses.push_back(cam_pose);
                }
                cam_path_temp.header.frame_id="map";
                    cam_path_temp.header.stamp=ros::Time::now();
                    campath=cam_path_temp;
            }
        }
    #endif
}


