import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class TableObjectDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.color_sub = rospy.Subscriber('/left_camera/color/image_raw', Image, self.color_callback)
        self.color_image = None

    def color_callback(self, msg):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_image()
        except CvBridgeError as e:
            rospy.logerr(f"Error converting color image: {e}")

    def process_image(self):
        if self.color_image is not None:
            # Convert the image to HSV color space
            hsv_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)

            # Define the HSV range for grey color (adjust as needed)
            lower_grey = np.array([0, 0, 50])
            upper_grey = np.array([180, 50, 200])

            # Create a mask for the grey table
            grey_mask = cv2.inRange(hsv_image, lower_grey, upper_grey)

            # Invert the grey mask to get the objects
            objects_mask = cv2.bitwise_not(grey_mask)

            # Create a mask to exclude the fixed base element (adjust as needed)
            exclude_mask = np.ones(objects_mask.shape, dtype=np.uint8) * 255
            height, width = objects_mask.shape[:2]
            # exclude_mask[:70, :80] = 0  # Left top corner
            exclude_mask[:30, width-120:] = 0  # Right top corner

            # Apply the exclude mask to the objects mask
            objects_mask = cv2.bitwise_and(objects_mask, objects_mask, mask=exclude_mask)

            # Check if there are any objects (non-zero pixels in the objects mask)
            if cv2.countNonZero(objects_mask) > 0:
                print("Objects detected on the table!")
                result_image = self.color_image.copy()
                contours, _ = cv2.findContours(objects_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(result_image, contours, -1, (0, 255, 0), 2)
                cv2.imshow('Objects on Table', result_image)
            else:
                print("No objects detected on the table.")

            cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('table_object_detector')
    detector = TableObjectDetector()
    rospy.spin()
    cv2.destroyAllWindows()
