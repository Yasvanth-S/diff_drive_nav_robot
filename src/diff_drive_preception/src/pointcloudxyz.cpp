#include <bits/stdc++.h>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "sensor_msgs/point_field_conversion.h"
#include "sensor_msgs/image_encodings.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "image_geometry/pinhole_camera_model.h"

#include "depth_traits.h"
#include "depth_conversion.h"

using namespace std;
namespace enc = sensor_msgs::image_encodings;

class pointcloud 
{
    private:
        ros::Publisher pcd_pub;
        ros::NodeHandle nh;
        shared_ptr<image_transport::ImageTransport> it;
        image_transport::CameraSubscriber image_sub;
        image_geometry::PinholeCameraModel model;
    public:
        pointcloud()
        {
            it.reset(new image_transport::ImageTransport(nh));
            pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1000);
            image_sub = it->subscribeCamera("/camera/depth/image_raw", 1000, &pointcloud::image_cb,this);
        }

    void image_cb(const sensor_msgs::Image::ConstPtr & img, const sensor_msgs::CameraInfoConstPtr & info)
    {
        sensor_msgs::PointCloud2::Ptr pcd(new sensor_msgs::PointCloud2);
        pcd->header = img->header;
        pcd->height = img->height;
        pcd->width = img->width;
        pcd->is_dense = false;
        pcd->is_bigendian = false;
        sensor_msgs::PointCloud2Modifier pcd_modifier(* pcd);
        pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
        model.fromCameraInfo(info);
        if(img->encoding == enc::TYPE_16UC1 || img->encoding == enc::MONO16)
        {
            depth_conversion::convert<uint16_t>(img,pcd,model);
        }
        else if(img->encoding == enc::TYPE_32FC1)
        {
            depth_conversion::convert<float>(img,pcd,model);
        }
        else
        {
            ROS_WARN_THROTTLE(5, "Depth Image has unsupported encoding [%s]", img->encoding.c_str());
            return;
        }
        pcd_pub.publish(pcd);
        cv_bridge::CvImageConstPtr cv_b;
        try
        {
            cv_b = cv_bridge::toCvShare(img);
        }
        catch(cv_bridge::Exception e)
        {
            ROS_ERROR("CV exception : %s", e.what());
        }
        cv::Mat mono8 = cv::Mat(cv_b->image.size(), CV_8UC1);
        cv::convertScaleAbs(cv_b->image,mono8, 100,0.0);
        cv::namedWindow("depth_image", cv::WINDOW_AUTOSIZE);
        cv::imshow("depth_image", cv_b->image);
        cv::waitKey(3);

    }
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "pointcloud");
    pointcloud pcd;
    ros::spin();
}




