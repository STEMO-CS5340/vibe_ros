#pragma once

#include <libvibe++/ViBe.h>
#include <libvibe++/distances/Manhattan.h>
#include <libvibe++/system/types.h>

#include <stereo_processor.h>

#include <opencv2/opencv.hpp>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "ros_publisher.hpp"

typedef ViBe::ViBeSequential<3, ViBe::Manhattan<3>> vibe_t;

class ViBeManager :  public StereoProcessor{

public:

    ViBeManager() : StereoProcessor(3,
        "/zed/zed_node/left/image_rect_color",
        "/zedm/zed_node/left/image_rect_color",
        "/zed/zed_node/left/camera_info",
        "/zedm/zed_node/left/camera_info"
    ), initialised(false){
        
        ros::NodeHandle nh_local("~");
        pub = new StereoCameraPublisher(nh_local);
        sys1 = nullptr;
        sys2 = nullptr;

    }

    ~ViBeManager(){
        if(sys1)
            delete sys1;

        if(sys2)
            delete sys2;

        cvDestroyAllWindows();
    }

    

private:
    vibe_t *sys1, *sys2;
    bool initialised;

    StereoCameraPublisher* pub;

    void imageCallback(	const sensor_msgs::ImageConstPtr l_image_msg,
								const sensor_msgs::ImageConstPtr r_image_msg,
								const sensor_msgs::CameraInfoConstPtr l_info_msg,
								const sensor_msgs::CameraInfoConstPtr r_info_msg);

};


void ViBeManager::imageCallback(	const sensor_msgs::ImageConstPtr l_image_msg,
								const sensor_msgs::ImageConstPtr r_image_msg,
								const sensor_msgs::CameraInfoConstPtr l_info_msg,
								const sensor_msgs::CameraInfoConstPtr r_info_msg)
{
    static int count = 0;
    std::cout << "imageCallback() " << count++ << std::endl;

    auto cvImage_l = cv_bridge::toCvShare(l_image_msg, "bgr8"); // CvImageConstPtr 
	auto cvImage_r = cv_bridge::toCvShare(r_image_msg, "bgr8");


	cv::Mat cv_leftImg_source = cvImage_l->image;
	cv::Mat cv_rightImg_source = cvImage_r->image;

    if (!initialised){
        
        if (count < 5)
            return;

        if (!sys1)
            sys1 = new vibe_t(l_image_msg->height, l_image_msg->width, cv_leftImg_source.data);
        if (!sys2)
            sys2 = new vibe_t(r_image_msg->height, r_image_msg->width, cv_rightImg_source.data);

        initialised = true;
        return;
    }

    cv::Mat segmentationMap1(l_image_msg->height, l_image_msg->width, CV_8UC1);
    cv::Mat segmentationMap2(r_image_msg->height, r_image_msg->width, CV_8UC1);

    sys1->segmentation(cv_leftImg_source.data, segmentationMap1.data);
    sys1->update(cv_leftImg_source.data, segmentationMap1.data);

    sys2->segmentation(cv_rightImg_source.data, segmentationMap2.data);
    sys2->update(cv_rightImg_source.data, segmentationMap2.data);


    /* Post-processing: 3x3 median filter. */
    cv::medianBlur(segmentationMap1, segmentationMap1, 3);
    cv::medianBlur(segmentationMap2, segmentationMap2, 3);

    // Publish
    sensor_msgs::CameraInfo left_info, right_info;

    left_info = *l_info_msg;
    right_info = *r_info_msg;

    pub->publish(segmentationMap1, segmentationMap2, "mono8",left_info, right_info, l_info_msg->header.stamp);

    // cv::imshow("Input video 1", cv_leftImg_source);
    // cv::imshow("Segmentation by ViBe 1", segmentationMap1);
    // cv::imshow("Segmentation by ViBe 2", segmentationMap2);

    // cvWaitKey(1);
}