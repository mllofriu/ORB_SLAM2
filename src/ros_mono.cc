/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include "include/System.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "orb_slam2/Frame.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

ORB_SLAM2::System* mpSLAM;
bool debug_view;

ros::Subscriber frame_sub;
message_filters::Subscriber<sensor_msgs::Image > * image_filter_sub;
message_filters::Subscriber<orb_slam2::Frame >  * frame_filter_sub;
message_filters::TimeSynchronizer<Image, orb_slam2::Frame > * msg_sync;

void grabImageAndFeatures(const sensor_msgs::ImageConstPtr& image,
                          const orb_slam2::FrameConstPtr & frame)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec(), *frame);
}

void grabFeatures(const orb_slam2::FrameConstPtr & frame)
{
    // If no features, just pass on a blank image. It has heigh rows and width columns
    cv::Mat emptyMat = cv::Mat(frame->height,frame->width, CV_8U);
    mpSLAM->TrackMonocular(emptyMat,frame->header.stamp.toSec(), *frame);
//    ROS_INFO("Features processed");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    mpSLAM = new ORB_SLAM2::System(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ros::NodeHandle nh("~");

    nh.param("debug_view", debug_view, false);

    if (debug_view){
        ROS_INFO("Subscribing to images and features");
        image_filter_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, "image", 1);
        frame_filter_sub = new message_filters::Subscriber<orb_slam2::Frame>(nh, "features", 1);
        msg_sync = new message_filters::TimeSynchronizer<Image, orb_slam2::Frame>(*image_filter_sub, *frame_filter_sub, 10);
        msg_sync->registerCallback(boost::bind(&grabImageAndFeatures, _1, _2));
    } else {
        ROS_INFO("Subscribing to features only");
        frame_sub = nh.subscribe("features", 1, &grabFeatures);
    }

    ros::spin();

    // Stop all threads
    mpSLAM->Shutdown();

    // Save camera trajectory
    mpSLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}




