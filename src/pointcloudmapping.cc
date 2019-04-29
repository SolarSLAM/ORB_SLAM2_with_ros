/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/projection_matrix.h>
#include "Converter.h"

#include <boost/make_shared.hpp>
namespace ORB_SLAM2
{
    PointCloudMapping::PointCloudMapping(shared_ptr<DataPub> pDataPub):mpDataPub(pDataPub)
    {
        voxel.setLeafSize( 0.04, 0.04, 0.04);
        globalMap = boost::make_shared< PointCloud >( );
        viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
    }

    void PointCloudMapping::shutdown()
    {
        {
            unique_lock<mutex> lck(shutDownMutex);
            shutDownFlag = true;
            keyFrameUpdated.notify_one();
        }
        viewerThread->join();
    }

    void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
    {
        cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
        unique_lock<mutex> lck(keyframeMutex);
        keyframes.push_back( kf );
        colorImgs.push_back( color.clone() );
        depthImgs.push_back( depth.clone() );

        keyFrameUpdated.notify_one();
    }

    pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
    {
        PointCloud::Ptr tmp( new PointCloud() );
        // point cloud is null ptr
        for ( int m=0; m<depth.rows; m+=3 )
        {
            for ( int n=0; n<depth.cols; n+=3 )
            {
                float d = depth.ptr<float>(m)[n];
                if (d < 0.01 || d>10)
                    continue;
                PointT p;
                p.z = d;
                p.x = ( n - kf->cx) * p.z / kf->fx;
                p.y = ( m - kf->cy) * p.z / kf->fy;

                if(color.type()==24)
                {
                    p.b = color.ptr<uchar>(m)[n*4];
                    p.g = color.ptr<uchar>(m)[n*4+1];
                    p.r = color.ptr<uchar>(m)[n*4+2];
                }
                else
                {
                    p.b = color.ptr<uchar>(m)[n*3];
                    p.g = color.ptr<uchar>(m)[n*3+1];
                    p.r = color.ptr<uchar>(m)[n*3+2];
                }


                tmp->points.push_back(p);

            }
        }
        PointCloud::Ptr cloud(new PointCloud);
        Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
        pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
        PointCloud::Ptr cloud_final(new PointCloud);
        Eigen::Isometry3d M = mpDataPub->getTransformation();
        pcl::transformPointCloud( *cloud, *cloud_final, M.matrix());
        cloud_final->is_dense = false;
        cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<"类型"<<color.type()<<endl;
        return cloud_final;
    }


    void PointCloudMapping::viewer()
    {
        pcl::visualization::CloudViewer viewer("viewer");
        while(1)
        {
            {
                unique_lock<mutex> lck_shutdown( shutDownMutex );
                if (shutDownFlag)
                {
                    break;
                }
            }
            {
                unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
                keyFrameUpdated.wait( lck_keyframeUpdated );
            }

            // keyframe is updated
            size_t N=0;
            {
                unique_lock<mutex> lck( keyframeMutex );
                N = keyframes.size();
            }
            PointCloud::Ptr curPC(new PointCloud);
            for ( size_t i=lastKeyframeSize; i<N ; i++ )
            {
                PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
                *globalMap += *p;
                *curPC += *p;
            }
            mpDataPub->getPointCloud(curPC);
            PointCloud::Ptr tmp(new PointCloud());
            voxel.setInputCloud( globalMap );
            voxel.filter( *tmp );
            globalMap->swap( *tmp );

            viewer.showCloud( globalMap );
            cout << "show global map, size=" << globalMap->points.size() << endl;
            lastKeyframeSize = N;
        }
    }
}

