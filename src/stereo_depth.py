import rospy
import cv2
import numpy as np
import os, rospkg
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
from numpy.linalg import inv
from std_msgs.msg import Header
import struct

bridge = CvBridge()
global left_image, right_image
max_disparity = 128
stereoProcessor = cv2.StereoSGBM_create(0, max_disparity, 5)
frame_count = 0
frame_index = rospy.get_param('frame_index')



def left_rgb_callback(data):
    global left_image
    left_image1 = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    left_image = left_image1

def right_rgb_callback(data):
    global right_image, left_image, max_disparity, frame_count
    right_image1 = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    frame_count += 1
    right_image = right_image1

    grayL = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

    # stereo = cv2.StereoBM_create(numDisparities=128, blockSize=15)

    disparity = stereoProcessor.compute(grayL, grayR)
    cv2.filterSpeckles(disparity, 0, 50, max_disparity)
    # disparity = stereo.compute(grayL, grayR).astype(np.float32)
    _, disparity = cv2.threshold(disparity, 0, max_disparity * 16, cv2.THRESH_TOZERO)
    # print(disparity.shape, depth.shape)

    pub_disparity = bridge.cv2_to_imgmsg(disparity, encoding = 'passthrough')
    pub.publish(pub_disparity)

    if frame_count == frame_index:

        h, w = grayL.shape[:2]
        focal_length = 2.8
        Q = np.float32([[1, 0, 0, -w/2.0],
                        [0,-1, 0,  h/2.0],
                        [0, 0, 0, -focal_length],
                        [0, 0, 1, 0]])
        # inv_disparity = inv(disparity)

        points_3D = cv2.reprojectImageTo3D(disparity, Q)
        colors = cv2.cvtColor(left_image, cv2.COLOR_BGR2RGB)
        mask_map = disparity > disparity.min()
        out_points = points_3D[mask_map]
        out_colors = colors[mask_map]
        out_colors = out_colors.reshape(-1,3)
        #create pointcloud
        a =255
        points_w_c = []
        print
        for i in range(len(out_points)):
            pt = out_points[i]
            color = out_colors[i]
            rgba = struct.unpack('I', struct.pack('BBBB', int(color[2]),int(color[1]), int(color[0]), a))[0]
            points_w_c.append([pt[0], pt[1], pt[2], rgba])

        header = Header()
        header.frame_id = 'map'

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgba', 12, PointField.FLOAT32, 1)]

        pc2 = point_cloud2.create_cloud(header, fields, points_w_c)

        # pc2.header.stamp = rospy.Time.now()
        # pc_pub.publish(pc2)

        pc2.header.stamp = rospy.Time.now()
        pc_pub.publish(pc2)
        rospy.sleep(0.5)









if __name__ == '__main__':
    rospy.init_node('stereo_depth', anonymous=True)
    rospy.Subscriber('/zed/zed_node/left/image_rect_color', Image, left_rgb_callback)
    rospy.Subscriber('/zed/zed_node/right/image_rect_color', Image, right_rgb_callback)
    pub = rospy.Publisher('/depth_topic', Image, queue_size = 10)
    pc_pub = rospy.Publisher('/pointcloud2_topic', PointCloud2, queue_size =10)
    try:
        rospy.spin()
    except KeyBoardInterrupt:
        print("Shutting Down!")
