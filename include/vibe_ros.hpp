#pragma once

#include <libvibe++/ViBe.h>
#include <libvibe++/distances/Manhattan.h>
#include <libvibe++/system/types.h>

#include <stereo_processor.h>

#include <opencv2/opencv.hpp>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

typedef ViBe::ViBeSequential<3, ViBe::Manhattan<3>> vibe_t;

class ViBeManager :  public StereoProcessor{

public:

    ViBeManager() : StereoProcessor(3,
        "/zedm/zed_node/left/image_rect_color",
        "/zedm/zed_node/left/image_rect_color",
        "/zedm/zed_node/left/camera_info",
        "/zedm/zed_node/left/camera_info"
    ), initialised(false){

    }

    ~ViBeManager(){
        if(sys)
            delete sys;

        cvDestroyAllWindows();
    }

    

private:
    vibe_t *sys;
    bool initialised;

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
	// auto cvImage_r = cv_bridge::toCvShare(r_image_msg, "mono8");


	cv::Mat cv_leftImg_source = cvImage_l->image;
	// cv::Mat cv_rightImg_source = cvImage_r->image;

    if (!initialised){

        if (count < 20)
            return;
        sys = new vibe_t(l_image_msg->height, l_image_msg->width, cv_leftImg_source.data);
        initialised = true;
        return;
    }

    cv::Mat segmentationMap(l_image_msg->height, l_image_msg->width, CV_8UC1);

    sys->segmentation(cv_leftImg_source.data, segmentationMap.data);
    sys->update(cv_leftImg_source.data, segmentationMap.data);


    /* Post-processing: 3x3 median filter. */
    cv::medianBlur(segmentationMap, segmentationMap, 3);

    cv::imshow("Input video", cv_leftImg_source);
    cv::imshow("Segmentation by ViBe", segmentationMap);

    cvWaitKey(1);
}