#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs import point_cloud2
import imageio.v3 as iio
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
import cv2
import struct


rospy.init_node("depth", anonymous=True)
pub = rospy.Publisher("pointcloud", PointCloud2, queue_size=10)
rate = rospy.Rate(100)
def depth_img(msg):
    FX_DEPTH = 587.1749125558551
    FY_DEPTH = 587.1749125558551
    CX_DEPTH = 360.5
    CY_DEPTH = 340.5
    cv_bridge = CvBridge()
    dep = cv_bridge.imgmsg_to_cv2(msg, 'passthrough')
    pcd2 = PointCloud2()
    pcd2.header.stamp = rospy.Time.now()
    pcd2.header.frame_id = 'camera_frame'
    pcd2.height, pcd2.width = dep.shape
    # print(f"Image resolution: {dep.shape}")
    # print(f"Data type: {dep.dtype}")
    # print(f"Min value: {np.nanmin(dep)}")
    # print(f"Max value: {np.nanmax(dep)}")
    pcd = []
    height, width = dep.shape
    # slow
    for i in range(height):  
        for j in range(width):
            z = dep[i][j]
            x = (j - CX_DEPTH) * z / FX_DEPTH
            y = (i - CY_DEPTH) * z / FY_DEPTH
            rgb = 4294967295
            pcd.append([x,y,z,rgb])
    pcd2.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),
                    PointField('rgba', 12, PointField.UINT32,1)]
    pc2 = point_cloud2.create_cloud(pcd2.header, pcd2.fields, pcd)
    pub.publish(pc2)
    depth_intensity = np.array(256*dep / 0x0fff,dtype=float)
    cv2.imshow('depth_intensity',depth_intensity)
    cv2.waitKey(10)
    rate.sleep()


def depth():
    rospy.Subscriber("/camera/depth/image_raw", Image, depth_img, queue_size=10)
    rospy.spin()

if __name__ == "__main__":
    depth()
