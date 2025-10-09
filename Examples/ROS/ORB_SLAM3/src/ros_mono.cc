/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include<ros/spinner.h>
#include<sensor_msgs/CameraInfo.h>
#include<sensor_msgs/Image.h>
#include<sensor_msgs/PointCloud2.h>
#include<nav_msgs/Path.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_broadcaster.h>
#include<Eigen/Dense>

#include <cv_bridge/cv_bridge.h>
#include<message_filters/subscriber.h>
#include<message_filters/time_synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>
#include<opencv2/core/eigen.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ros::NodeHandle nh;
    ros::Publisher pub_tcw,pub_camerapath,pub_odom;
    size_t mcounter = 0;
    nav_msgs::Path camerapath;
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM),nh("~"){
        pub_tcw= nh.advertise<geometry_msgs::PoseStamped> ("CameraPose", 10); 
	    pub_odom= nh.advertise<nav_msgs::Odometry> ("Odometry", 10); 
	    pub_camerapath= nh.advertise<nav_msgs::Path> ("Path", 10);
    }

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat Tcw;
    Sophus::SE3f Tcw_SE3f = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    Eigen::Matrix4f Tcw_Matrix = Tcw_SE3f.matrix();
    cv::eigen2cv(Tcw_Matrix,Tcw);

    if (!Tcw.empty())
	{
					cv::Mat Twc =Tcw.inv();
					cv::Mat RWC= Twc.rowRange(0,3).colRange(0,3);  
					cv::Mat tWC=  Twc.rowRange(0,3).col(3);

					Eigen::Matrix<double,3,3> eigMat ;
					eigMat <<RWC.at<float>(0,0),RWC.at<float>(0,1),RWC.at<float>(0,2),
									RWC.at<float>(1,0),RWC.at<float>(1,1),RWC.at<float>(1,2),
									RWC.at<float>(2,0),RWC.at<float>(2,1),RWC.at<float>(2,2);
					Eigen::Quaterniond q(eigMat);
 
				 geometry_msgs::PoseStamped tcw_msg; 					
                 tcw_msg.pose.position.x=tWC.at<float>(0);
                 tcw_msg.pose.position.y=tWC.at<float>(1);			 
                 tcw_msg.pose.position.z=tWC.at<float>(2);
				 
				tcw_msg.pose.orientation.x=q.x();
				tcw_msg.pose.orientation.y=q.y();
				tcw_msg.pose.orientation.z=q.z();
				tcw_msg.pose.orientation.w=q.w();
				   
				  std_msgs::Header header ;
				  header.stamp =msg->header.stamp;
				  header.seq = msg->header.seq;
				  header.frame_id="world";
 
				 tcw_msg.header=header;
				 
				 // odometry information
				 nav_msgs::Odometry odom_msg;
				odom_msg.pose.pose.position.x=tWC.at<float>(0);
                 odom_msg.pose.pose.position.y=tWC.at<float>(1);			 
                 odom_msg.pose.pose.position.z=tWC.at<float>(2);
				 
				odom_msg.pose.pose.orientation.x=q.x();
				odom_msg.pose.pose.orientation.y=q.y();
				odom_msg.pose.pose.orientation.z=q.z();
				odom_msg.pose.pose.orientation.w=q.w();
				
				odom_msg.header=header;
				odom_msg.child_frame_id="base_link"; 
				
				 camerapath.header =header;
				 camerapath.poses.push_back(tcw_msg);
				  pub_odom.publish(odom_msg);	  
				 pub_camerapath.publish(camerapath);  //相机轨迹
				pub_tcw.publish(tcw_msg);	                      //Tcw位姿信息
				 
	}
	else
	{
	  cout<<"Twc is empty ..."<<endl;
	}
}


