import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import cv2
import os
import numpy as np

class Nodo(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)

        # Publishers
        self.pub = rospy.Publisher('imagetimer', Image,queue_size=10)

        # Subscribers
        rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

        rospy.sleep(2)
        
    def callback(self, msg):
        rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg, "bgr8")

    def start(self):
        rospy.loginfo("Timing images")
        
        if self.image is not None:
            
            image_copy = self.image.copy()
            # Convert the image to grayscale
            hsv = cv2.cvtColor(image_copy, cv2.COLOR_BGR2HSV)

            # Define range for red color in HSV
            lower_red = np.array([0,120,70])
            upper_red = np.array([10,255,255])

            # Threshold the HSV image to get only red colors
            mask = cv2.inRange(hsv, lower_red, upper_red)

                # Morphological operations for refinement
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # Draw contours on the original image
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 1000:  # Adjust this value as needed
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(image_copy, (x, y), (x+w, y+h), (0, 255, 0), 2)


            # Configure depth and color streams
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth)
            config.enable_stream(rs.stream.color)
            
            # Start streaming
            pipeline.start(config)

            try:
                # Capture frames from the camera
                frames = pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()

                # Get intrinsics for the depth frame
                intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

                # Create a point cloud object
                pc = rs.pointcloud()
                points = pc.calculate(depth_frame)
                vtx = np.asanyarray(points.get_vertices())

                # Filter points based on the contour
                filtered_points = []
                for i, point in enumerate(vtx):
                    px, py = rs.rs2_project_point_to_pixel(intrinsics, [point[0], point[1], point[2]])
                    if x <= px < x + w and y <= py < y + h:
                        filtered_points.append(point)

                # filtered_points now contains the 3D points within your contour

            finally:
                pipeline.stop()
                        
            # # Display the result
            # cv2.imshow('Detected Line', image_copy)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
        
        
        
            
if __name__ == '__main__':
    rospy.init_node("image_processing", anonymous=True)
    my_node = Nodo()
    my_node.start()