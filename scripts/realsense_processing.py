#!/usr/bin/env python3
import rospy
import tf2_ros
from laser_assembler.srv import (
    AssembleScans2,
    AssembleScans2Request,
    AssembleScans2Response,
)
from rospy.timer import TimerEvent
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

# from box_grasping.utils.tf_helper import init_tf_tree
from first_demo.srv import PCLFwdStatus, PCLFwdStatusRequest, PCLFwdStatusResponse
# from utils import init_tf_tree

class RealSenseProcessor:
    # Class that provides services to easily interface with an attached RealSense camera
    def __init__(self) -> None:
        rospy.init_node("realsense_processing", anonymous=True)
        stitcher_input_topic = rospy.get_param('~stitcher_input_topic')
        pcd_topic = rospy.get_param('~pointcloud_topic')
        scan_topic = rospy.get_param('~scan_topic')
        laser_topic = rospy.get_param('~laser_topic')
        
        rospy.wait_for_service(laser_topic)

        self.assemble_pcds = rospy.ServiceProxy(laser_topic, AssembleScans2)
        
        
        # Self-created service for stitching all clouds, reseponse is the stitched point clouds
        stitch_srv = rospy.Service(scan_topic, AssembleScans2, self.stitch_pcds_callback)

        # self.tf_listener = init_tf_tree()

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30))
        self.tl = tf2_ros.TransformListener(self.tf_buffer)


        
        
        self.pcd_forward_pub = rospy.Publisher(stitcher_input_topic, PointCloud2, queue_size=1)
        
        
        # Getting the most recent point cloud data which is transformed into the robot base already
        # And store it into the self.pcl_latest
        self.pcl_latest = None
        self.pcd_reader_sub = rospy.Subscriber(
            pcd_topic, PointCloud2, self.pcd_callback
        )
        
        while self.pcl_latest is None:
            rospy.loginfo("Waiting for point cloud")
            rospy.sleep(rospy.Duration(nsecs=200))


        # Actual forwarder
        # Publish the assembled point cloud data and nodelet is suppoed to process this in the launch file
        fwder = rospy.Timer(rospy.Duration(secs=2, nsecs=0), self.fwd_latest_pcd_timer_callback)


        
        
        
        
        rospy.spin()





    def stitch_pcds_callback(self, req: AssembleScans2Request) -> AssembleScans2Response:
        """
        assembles all point cloud data
        """
        
        # This should call laser assembler with these times to stitch the relevant point clouds
        # The idea here is that we collect clouds through some known time elsewhere which
        # are auto-published to the assembler topic, and then the call here stitches all of
        # the clouds together

        # laser_assembler should (i think) track the transforms on its own, but maybe only
        # for some set time. IE. i don't know if it explicitly tracks the best matching tf
        # for each stored frame or if it just keeps all tfs in a buffer

        # Need to add the laser_assembler package to catkin as well

        stitched_pcl = self.assemble_pcds(req.begin, req.end)
        return stitched_pcl



    def pcd_callback(self, data: PointCloud2) -> None:
        """
        Stores the most recently received pcl ros message
        """

        try:
            tf = self.tf_buffer.lookup_transform(
                "base_link", 
                data.header.frame_id, 
                data.header.stamp, 
                rospy.Duration(0.1)
            )
            # rospy.logwarn(f'Transform from "base_link to {data.header.frame_id} found! And the stamp is: {data.header.stamp}"')
        except:
            # TODO: this seems to always occur once, why?
            #   Shouldn't be a major issue but is annoying nonetheless
            rospy.logwarn(f'Transform from "base_link to {data.header.frame_id} not found!"')
            rospy.loginfo("Pcd dropped due to missing transform")
            return

        self.pcl_latest = data
        



        
    
    
    def fwd_latest_pcd_timer_callback(self, event: TimerEvent):
        """
        Publishes the latest stored recently pcl
        """
        # if rospy.get_param("/realsense_processing/pcl_fwd_status"):
        self.pcd_forward_pub.publish(self.pcl_latest)



if __name__ == "__main__":
    try:
        grasper = RealSenseProcessor()
    except KeyboardInterrupt:
        pass