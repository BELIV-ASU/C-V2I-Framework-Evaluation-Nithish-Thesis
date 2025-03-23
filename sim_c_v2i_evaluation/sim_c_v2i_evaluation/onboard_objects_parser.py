import rclpy
from rclpy.node import Node

from autoware_auto_perception_msgs.msg import DetectedObjects

import csv

class ObjectsParserNode(Node):
    def __init__(self):
        super().__init__("object_parser_node")

        self.csv_file_groundtruth = \
            open('/home/nithish/autoware/src/universe/autoware.universe/perception/sim_c_v2i_evaluation/data/onboard_objects/ground_truth_objects.csv',\
                              'a', newline='')
        self.csv_writer_groundtruth = csv.writer(self.csv_file_groundtruth)
        self.objects_data_groundtruth = []

        self.csv_file_onboard = \
            open('/home/nithish/autoware/src/universe/autoware.universe/perception/sim_c_v2i_evaluation/data/onboard_objects/onboard_objects.csv',\
                              'a', newline='')
        self.csv_writer_onboard = csv.writer(self.csv_file_onboard)
        self.objects_data_onboard = []

        self.groundtruth_objects = DetectedObjects()

        self.groundtruth_objects_subscription = self.create_subscription(DetectedObjects, \
                                                "/perception/object_recognition/detection/carla/autoware_objects", self.groundtruth_objects_callback, 10)
        
        self.onboard_objects_subscription = self.create_subscription(DetectedObjects, \
                                                "/perception/object_recognition/detection/centerpoint/objects", self.objects_parser_callback, 10)
                

    def objects_parser_callback(self, objects_msg):
        if objects_msg.objects != []:
            for obj in objects_msg.objects:
                self.objects_data_onboard.extend([obj.kinematics.pose_with_covariance.pose.position.x,
                                        obj.kinematics.pose_with_covariance.pose.position.y,
                                        obj.kinematics.pose_with_covariance.pose.position.z,
                                        obj.kinematics.pose_with_covariance.pose.orientation.x,
                                        obj.kinematics.pose_with_covariance.pose.orientation.y,
                                        obj.kinematics.pose_with_covariance.pose.orientation.z,
                                        obj.kinematics.pose_with_covariance.pose.orientation.w,
                                        obj.shape.dimensions.x,
                                        obj.shape.dimensions.y,
                                        obj.shape.dimensions.z])
            self.csv_writer_onboard.writerow(self.objects_data_onboard)
            self.objects_data_onboard = []

            for obj in self.groundtruth_objects.objects:
                self.objects_data_groundtruth.extend([obj.kinematics.pose_with_covariance.pose.position.x,
                                        obj.kinematics.pose_with_covariance.pose.position.y,
                                        obj.kinematics.pose_with_covariance.pose.position.z,
                                        obj.kinematics.pose_with_covariance.pose.orientation.x,
                                        obj.kinematics.pose_with_covariance.pose.orientation.y,
                                        obj.kinematics.pose_with_covariance.pose.orientation.z,
                                        obj.kinematics.pose_with_covariance.pose.orientation.w,
                                        obj.shape.dimensions.x,
                                        obj.shape.dimensions.y,
                                        obj.shape.dimensions.z])
            self.csv_writer_groundtruth.writerow(self.objects_data_groundtruth)
            self.objects_data_groundtruth = []

    
    def groundtruth_objects_callback(self, groundtruth_objects_msg):
        self.groundtruth_objects = groundtruth_objects_msg



def main(args=None):
    rclpy.init(args=args)
    node = ObjectsParserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()