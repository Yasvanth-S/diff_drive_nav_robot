#pragma once 
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <limits>

#include "depth_traits.h"

namespace depth_conversion
{
    using pointcloud2 = sensor_msgs::PointCloud2;

    template <typename T>
    inline void convert(const sensor_msgs::ImageConstPtr & depth, pointcloud2::Ptr& cloud,
                        const image_geometry::PinholeCameraModel & model, double range_max = 0.0)
    {
        float center_x = model.cx();
        float center_y = model.cy();
        
        double unit_scaling = depth_traits::DepthTraits<T>::toMeters(T(1));
        float constant_x = unit_scaling / model.fx();
        float constant_y = unit_scaling / model.fy();
        float bad_point = std::numeric_limits<float>::quiet_NaN();

        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");
        const T* depth_row = reinterpret_cast<const T*>(&depth->data[0]);
        int row_step = depth->step / sizeof(T);
        for(int v = 0; v < (int)cloud->height; ++v, depth_row+=row_step)
        {
            for(int u = 0; u < (int)cloud->width; ++u, ++iter_x, ++iter_y, ++iter_z)
            {
                T dep = depth_row[u];
                if(!depth_traits::DepthTraits<T>::valid(dep))
                {
                    if(range_max != 0.0)
                    {
                        dep = depth_traits::DepthTraits<T>::fromMeters(range_max);
                    }
                    else
                    {
                        *iter_x = *iter_y = *iter_z = bad_point;
                        continue;
                    }
                }
                    *iter_x = (u - center_x) * dep * constant_x;
                    *iter_y = (v - center_y) * dep * constant_y;
                    *iter_z = depth_traits::DepthTraits<T>::toMeters(dep);
            }
            
        }
    }
}