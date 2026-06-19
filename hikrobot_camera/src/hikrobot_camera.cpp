#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include "hikrobot_camera.hpp"

// 剪裁掉照片和雷达没有重合的视角，去除多余像素可以使rosbag包变小
#define FIT_LIDAR_CUT_IMAGE false
#if FIT_LIDAR_CUT_IMAGE
    #define FIT_min_x 420
    #define FIT_min_y 70
    #define FIT_max_x 2450
    #define FIT_max_y 2000
#endif 

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    //********** variables    **********/
    cv::Mat src;

    //********** rosnode init **********/
    ros::init(argc, argv, "hikrobot_camera");
    ros::NodeHandle hikrobot_camera;
    camera::Camera MVS_cap(hikrobot_camera);

    //********** rosnode init **********/
    image_transport::ImageTransport main_cam_image(hikrobot_camera);
    image_transport::CameraPublisher image_pub = main_cam_image.advertiseCamera("/hikrobot_camera/rgb", 1000);

    sensor_msgs::Image image_msg;
    sensor_msgs::CameraInfo camera_info_msg;
    cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;

    //********** 填写相机内参（来自ost.yaml）**********//
    camera_info_msg.width  = 3200;
    camera_info_msg.height = 1944;
    camera_info_msg.distortion_model = "plumb_bob";

    camera_info_msg.K = {4732.80839, 0.0,       1484.88015,
                         0.0,        4728.1779,  1149.38858,
                         0.0,        0.0,        1.0};

    camera_info_msg.D = {0.076691, -0.169797, 0.004726, 0.006745, 0.0};

    camera_info_msg.R = {1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0};

    camera_info_msg.P = {4777.86426, 0.0,        1496.61659, 0.0,
                         0.0,        4781.83447,  1154.02226, 0.0,
                         0.0,        0.0,         1.0,        0.0};
    //**********************************************//

    //********** 10 Hz        **********/
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();

        MVS_cap.ReadImg(src);
        if (src.empty())
        {
            continue;
        }
#if FIT_LIDAR_CUT_IMAGE
        cv::Rect area(FIT_min_x, FIT_min_y, FIT_max_x-FIT_min_x, FIT_max_y-FIT_min_y);
        cv::Mat src_new = src(area);
        cv_ptr->image = src_new;
#else
        cv_ptr->image = src;
#endif
        image_msg = *(cv_ptr->toImageMsg());
        image_msg.header.stamp    = ros::Time::now();
        image_msg.header.frame_id = "hikrobot_camera";

        camera_info_msg.header.frame_id = image_msg.header.frame_id;
        camera_info_msg.header.stamp    = image_msg.header.stamp;
        image_pub.publish(image_msg, camera_info_msg);
    }
    return 0;
}