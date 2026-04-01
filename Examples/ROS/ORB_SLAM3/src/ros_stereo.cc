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

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"

#include <opencv2/core/eigen.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64.h>
#include "../../../include/Converter.h"
#include <nav_msgs/Path.h>
#include <Eigen/Dense>

// ========== 新增：点云发布与PCL相关头文件 ==========
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "../../../include/Atlas.h"
#include "../../../include/MapPoint.h"

using namespace std;

class ImageGrabber
{
public:
    ros::NodeHandle nh;
    ros::Publisher pub_pose;

    // ========== 新增：点云发布器与帧计数器 ==========
    ros::Publisher pub_pointcloud;
    int frame_counter;

#ifdef USE_SALIENCY_EKF
    ros::Publisher pub_saliency_vis;
    ros::Publisher pub_cov_norm_vo;
    ros::Publisher pub_path_2d; // 新增：2D轨迹发布器
    nav_msgs::Path path_2d;     // 新增：2D轨迹对象
#endif
    ImageGrabber(ORB_SLAM3::System *pSLAM) : mpSLAM(pSLAM), nh("~")
    {
        pub_pose = nh.advertise<geometry_msgs::PoseStamped>("CameraPose", 20);

        // ========== 新增：初始化点云发布器 (发布到 /orb_slam3/point_cloud) ==========
        pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("/orb_slam3/point_cloud", 5);
        frame_counter = 0;

#ifdef USE_SALIENCY_EKF
        pub_saliency_vis = nh.advertise<std_msgs::Float64>("saliency_visual", 10);
        pub_cov_norm_vo = nh.advertise<std_msgs::Float64>("cov_norm_vo", 10);

        // 新增：初始化2D轨迹发布器
        pub_path_2d = nh.advertise<nav_msgs::Path>("path_2d", 10);
        path_2d.header.frame_id = "map"; // 统一使用 map 坐标系便于 EKF 和 Rviz 融合
#endif
    }

    void GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight);
    void PublishPose(cv::Mat Tcw);

    // ========== 新增：发布全局点云的方法 ==========
    void PublishPointCloud(ros::Time stamp);

    ORB_SLAM3::System *mpSLAM;
    bool do_rectify;
    cv::Mat M1l, M2l, M1r, M2r;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "STEREO");
    ros::start();

    if (argc != 4)
    {
        cerr << endl
             << "Usage: rosrun ORB_SLAM3 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[3]);
    ss >> boolalpha >> igb.do_rectify;

    if (igb.do_rectify)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, igb.M1l, igb.M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, igb.M1r, igb.M2r);
    }

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/infra1/image_rect_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/infra2/image_rect_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo, &igb, _1, _2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // ========== 修复：只进行一次追踪 ==========
    Sophus::SE3f Tcw_SE3f;
    if (do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image, imLeft, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image, imRight, M1r, M2r, cv::INTER_LINEAR);
        Tcw_SE3f = mpSLAM->TrackStereo(imLeft, imRight, cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        Tcw_SE3f = mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, cv_ptrLeft->header.stamp.toSec());
    }

// #ifdef USE_SALIENCY_EKF
//     // 1. 获取刚刚在 Tracking 线程中算出的帧级显著性
//     float S_visual = mpSLAM->GetCurrentFrameSaliencyVisual();

//     // 2. 根据显著性计算观测噪声协方差范数 ||R_vo||
//     double R_base_vo = 0.01;
//     double k_vo = 3.2;
//     double R_vo_norm = R_base_vo * std::exp(k_vo * (1.0 - S_visual));

//     // 3. 发布
//     std_msgs::Float64 msg_s_vis;
//     msg_s_vis.data = S_visual;
//     pub_saliency_vis.publish(msg_s_vis);

//     std_msgs::Float64 msg_r_vo;
//     msg_r_vo.data = R_vo_norm;
//     pub_cov_norm_vo.publish(msg_r_vo);

//     if (!Tcw_SE3f.translation().hasNaN())
//     {
//         Sophus::SE3f Twc = Tcw_SE3f.inverse();
//         Eigen::Vector3f translation = Twc.translation();
//         Eigen::Matrix3f rotation = Twc.rotationMatrix();

//         geometry_msgs::PoseStamped pose_2d;
//         pose_2d.header.stamp = msgLeft->header.stamp;
//         pose_2d.header.frame_id = "map";

//         pose_2d.pose.position.x = translation.z();
//         pose_2d.pose.position.y = -translation.x();
//         pose_2d.pose.position.z = 0.0;

//         Eigen::Vector3f forward_dir = rotation * Eigen::Vector3f(0, 0, 1);
//         double yaw = atan2(-forward_dir.x(), forward_dir.z());
//         pose_2d.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

//         path_2d.poses.push_back(pose_2d);
//         path_2d.header.stamp = msgLeft->header.stamp;
//         pub_path_2d.publish(path_2d);
//     }
// #endif

    // 发布原有的3D位姿
    cv::Mat Tcw;
    Eigen::Matrix4f Tcw_Matrix = Tcw_SE3f.matrix();
    cv::eigen2cv(Tcw_Matrix, Tcw);
    PublishPose(Tcw);

    // ========== 新增：控制点云发布频率 (每5帧发布一次) ==========
    frame_counter++;
    if (frame_counter % 5 == 0)
    {
        PublishPointCloud(msgLeft->header.stamp);
    }

    std::chrono::milliseconds tSleep(1);
    std::this_thread::sleep_for(tSleep);
}

void ImageGrabber::PublishPose(cv::Mat Tcw)
{
    geometry_msgs::PoseStamped poseMSG;
    if (!Tcw.empty())
    {
        cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);
        vector<float> q = ORB_SLAM3::Converter::toQuaternion(Rwc);
        poseMSG.pose.position.x = -twc.at<float>(0);
        poseMSG.pose.position.y = -twc.at<float>(2);
        poseMSG.pose.position.z = twc.at<float>(1);
        poseMSG.pose.orientation.x = q[0];
        poseMSG.pose.orientation.y = q[1];
        poseMSG.pose.orientation.z = q[2];
        poseMSG.pose.orientation.w = q[3];
        poseMSG.header.frame_id = "map"; // 注意：如果你统一用 map，这里也可以改成 map
        poseMSG.header.stamp = ros::Time::now();
        pub_pose.publish(poseMSG);
    }
}

// ========== 新增：发布点云的函数实现 ==========
// ========== 修复后的发布点云函数 ==========
void ImageGrabber::PublishPointCloud(ros::Time stamp)
{
    // 确保 Atlas 初始化
    if (!mpSLAM->GetAtlas())
        return;

    // 获取所有活动的地图点
    std::vector<ORB_SLAM3::MapPoint *> vpMPs = mpSLAM->GetAtlas()->GetAllMapPoints();
    if (vpMPs.empty())
        return;

    // 创建 PCL 点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(vpMPs.size());

    for (size_t i = 0; i < vpMPs.size(); i++)
    {
        ORB_SLAM3::MapPoint *pMP = vpMPs[i];
        // 剔除坏点
        if (pMP && !pMP->isBad())
        {
            // 修复点：ORB-SLAM3 使用 Eigen::Vector3f 而非 cv::Mat 来表示三维坐标
            Eigen::Vector3f pos = pMP->GetWorldPos();
            pcl::PointXYZ p;

            // 坐标映射：ORB-SLAM3系 -> ROS标准Map系
            // 修复点：使用 () 来访问 Eigen 向量的元素
            p.x = pos(2);  // Z -> X
            p.y = -pos(0); // X -> -Y
            p.z = -pos(1); // Y -> -Z

            cloud->points.push_back(p);
        }
    }

    // 转为 ROS PointCloud2 消息并发布
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.stamp = stamp;
    cloud_msg.header.frame_id = "map"; // 必须与 2D 轨迹以及 octomap_server 配置的 frame_id 一致

    pub_pointcloud.publish(cloud_msg);
}