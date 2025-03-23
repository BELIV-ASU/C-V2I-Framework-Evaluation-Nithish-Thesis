import rclpy
from rclpy.node import Node

from autoware_auto_perception_msgs.msg import DetectedObjects

import csv

class ObjectsParserNode(Node):
    def __init__(self):
        super().__init__("object_parser_node")

        self.csv_file = open('/home/nithish/autoware/src/universe/autoware.universe/perception/sim_c_v2i_evaluation/data/ground_truth_objects.csv',\
                              'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.objects_data = []

        self.groundtruth_objects_subscription = self.create_subscription(DetectedObjects, \
                                                "/perception/object_recognition/detection/carla/autoware_objects", self.objects_parser_callback, 10)
    
    def objects_parser_callback(self, objects_msg):
        print("Ground truth objects: ", len(objects_msg.objects))
        for obj in objects_msg.objects:
            self.objects_data.extend([obj.kinematics.pose_with_covariance.pose.position.x,
                                     obj.kinematics.pose_with_covariance.pose.position.y,
                                     obj.kinematics.pose_with_covariance.pose.position.z,
                                     obj.kinematics.pose_with_covariance.pose.orientation.x,
                                     obj.kinematics.pose_with_covariance.pose.orientation.y,
                                     obj.kinematics.pose_with_covariance.pose.orientation.z,
                                     obj.kinematics.pose_with_covariance.pose.orientation.w])
        self.csv_writer.writerow(self.objects_data)
        self.objects_data = []



def main(args=None):
    rclpy.init(args=args)
    node = ObjectsParserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()