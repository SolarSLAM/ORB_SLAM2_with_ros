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
        DataPub::DataPub()
        {
            globalpc = boost::make_shared< PointCloud >();
            pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_output",1);
            pubThread = make_shared<thread>(bind(&DataPub::run,this));

        }
        void DataPub::shutdown()
        {
            {
                unique_lock<mutex> lck(shutdownMutex);
                shutdownFlag = true;
                datapubUpdated.notify_one();
            }
            pubThread->join();
        }

        void DataPub::getPointCloud(PointCloud::Ptr curPointCloud )
        {
            unique_lock<mutex> lck(datapubMutex);
            if(!isPubData)
                globalpc->swap(*curPointCloud);
            datapubUpdated.notify_one();
        }

        void DataPub::run()
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
                    while(ros::ok())
                    {
                        pcl_pub.publish(output);
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
                    isPubData = false;
                    cout<<"完成"<<globalpc->size()<<"发送"<<endl;
                }

            }
        }

}
