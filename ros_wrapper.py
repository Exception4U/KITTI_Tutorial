#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import message_filters

def callback(image, camera_param, velodyne):
	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(image, "bgr8")
	print "TUshar"
	print len(cv_image)
	print camera_param
	print len(velodyne)
    
    
def listener():
	rospy.init_node('listener', anonymous=True)

	l_image = message_filters.Subscriber("/kitti/camera_color_left/image_raw", Image )
	camera_info = message_filters.Subscriber("/kitti/camera_gray_left/camera_info", CameraInfo)
	lidar = message_filters.Subscriber("/kitti/velo/pointcloud", PointCloud2)
	ts = message_filters.TimeSynchronizer([l_image, camera_info, lidar], 10)
	ts.registerCallback(callback)
	rospy.spin()

if __name__ == '__main__':
	listener()
