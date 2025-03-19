import math

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from autoware_auto_perception_msgs.msg import DetectedObjects
from nav_msgs.msg import Odometry

import numpy as np
from scipy.spatial.transform import Rotation as R

import time

class FrameListener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.01, self.frame_transformation)

        self.infrastructure_objects_subscriber = self.create_subscription(DetectedObjects, \
                '/perception/object_recognition/detection/carom_frame_transformed/objects', self.infrastructure_objects_transformer, 10)
        
        self.transformed_objects_publisher = self.create_publisher(DetectedObjects, \
                '/perception/object_recognition/detection/carom_frame_transformed/objects2', 10)
        
        self.kinematic_state_subscriber = self.create_subscription(Odometry, \
                '/localization/kinematic_state', self.kinematic_state_updater, 10)

        self.x_tf, self.y_tf, self.z_tf = 0, 0, 0
        self.roll_tf, self.pitch_tf, self.yaw_tf = 0, 0, 0
        self.ox, self.oy, self.oz, self.ow = 1,1,1,1
        self.predicted_displacement_x, self.predicted_displacement_y = 0, 0

        self.start_time = 0
    
    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return yaw_z # in radians
    
    def frame_transformation(self):
        try:
            t = self.tf_buffer.lookup_transform('base_link', 'dummy_base_link', rclpy.time.Time())
            self.yaw_tf = self.euler_from_quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)
            self.ox, self.oy, self.oz, self.ow = t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w
            self.x_tf, self.y_tf, self.z_tf = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z 
            #print('yaw_tf', self.yaw_tf)
        except TransformException as ex:
            self.get_logger().info(f'Could not transform "base_link" to "camera_infrastructure_local_frame": {ex}')
            return 0
        
    def kinematic_state_updater(self, kinematic_state_msg):
        self.predicted_displacement_x = kinematic_state_msg.twist.twist.linear.x * 0.1
        self.predicted_displacement_y = kinematic_state_msg.twist.twist.linear.y * 0.1

    '''       
    def transform_camera_frame_to_base_frame(self, vehicle_position_camera_frame, vehicle_orientation_camera_frame):
        vehicle_position_camera = np.array([vehicle_position_camera_frame.x, vehicle_position_camera_frame.y, vehicle_position_camera_frame.z, 1])
        transformation_matrix = np.array([[np.cos(self.yaw_tf), -np.sin(self.yaw_tf), 0, self.x_tf-(math.copysign(1, self.x_tf)*self.predicted_displacement_x)],\
                                        [np.sin(self.yaw_tf), np.cos(self.yaw_tf), 0, self.y_tf-(math.copysign(1, self.y_tf)*self.predicted_displacement_y)],\
                                        [0, 0, 1, self.z_tf],\
                                        [0, 0, 0, 1]])
        transformed_position = np.dot(transformation_matrix, vehicle_position_camera.T).T + \
              np.array([(math.copysign(1, self.x_tf)*self.predicted_displacement_x), (math.copysign(1, self.y_tf)*self.predicted_displacement_y), 0, 0])
        vehicle_orientation_camera = np.array([vehicle_orientation_camera_frame.x, vehicle_orientation_camera_frame.y,\
                                                vehicle_orientation_camera_frame.z, vehicle_orientation_camera_frame.w])
        vehicle_orientation_base = np.array([self.ox, self.oy, self.oz, self.ow])
        r1 = R.from_quat(vehicle_orientation_camera)
        r2 = R.from_quat(vehicle_orientation_base)
        r3 = r2*r1
        transformed_orientation = r3.as_quat()

        return transformed_position, transformed_orientation
    '''

    def transform_camera_frame_to_base_frame(self, vehicle_position_camera_frame, vehicle_orientation_camera_frame):
        vehicle_position_camera = np.array([vehicle_position_camera_frame.x, vehicle_position_camera_frame.y, vehicle_position_camera_frame.z, 1])
        transformation_matrix = np.array([[np.cos(self.yaw_tf), -np.sin(self.yaw_tf), 0, self.x_tf],\
                                        [np.sin(self.yaw_tf), np.cos(self.yaw_tf), 0, self.y_tf],\
                                        [0, 0, 1, self.z_tf],\
                                        [0, 0, 0, 1]])
        transformed_position = np.dot(transformation_matrix, vehicle_position_camera.T)
        vehicle_orientation_camera = np.array([vehicle_orientation_camera_frame.x, vehicle_orientation_camera_frame.y,\
                                                vehicle_orientation_camera_frame.z, vehicle_orientation_camera_frame.w])
        vehicle_orientation_base = np.array([self.ox, self.oy, self.oz, self.ow])
        r1 = R.from_quat(vehicle_orientation_camera)
        r2 = R.from_quat(vehicle_orientation_base)
        r3 = r2*r1
        transformed_orientation = r3.as_quat()

        return transformed_position, transformed_orientation
    
    def infrastructure_objects_transformer(self, infrastructure_detection_msg): 
        self.start_time = time.time()

        infrastructure_detection_msg.header.frame_id = "base_link"
        for j, infrastructure_object in enumerate(infrastructure_detection_msg.objects):

            [infrastructure_object.kinematics.pose_with_covariance.pose.position.x,\
                infrastructure_object.kinematics.pose_with_covariance.pose.position.y,\
                infrastructure_object.kinematics.pose_with_covariance.pose.position.z, scale], \
                [infrastructure_object.kinematics.pose_with_covariance.pose.orientation.x, \
                infrastructure_object.kinematics.pose_with_covariance.pose.orientation.y, \
                infrastructure_object.kinematics.pose_with_covariance.pose.orientation.z, \
                infrastructure_object.kinematics.pose_with_covariance.pose.orientation.w] = \
                    self.transform_camera_frame_to_base_frame(infrastructure_object.kinematics.pose_with_covariance.pose.position,\
                                                               infrastructure_object.kinematics.pose_with_covariance.pose.orientation)
            
        time_difference = time.time() - self.start_time
        #print("Time taken for message format conversion (during subscribtion): ", time_difference*1e9, " ns")

        seconds = int((infrastructure_detection_msg.header.stamp.nanosec + int(time_difference*1e9))*1e-9)
        infrastructure_detection_msg.header.stamp.sec = infrastructure_detection_msg.header.stamp.sec + seconds
            
        infrastructure_detection_msg.header.stamp.nanosec = infrastructure_detection_msg.header.stamp.nanosec + int(time_difference*1e9) - seconds

        
        self.transformed_objects_publisher.publish(infrastructure_detection_msg)
        
        

          


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()