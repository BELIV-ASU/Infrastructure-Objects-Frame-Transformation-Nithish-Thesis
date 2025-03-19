import math

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from autoware_auto_perception_msgs.msg import DetectedObjects
from autoware_auto_perception_msgs.msg import DetectedObject
from autoware_auto_perception_msgs.msg import ObjectClassification

import pandas as pd
import numpy as np

import time

class FrameListener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Call on_timer function every second
        self.timer = self.create_timer(0.005, self.frame_transformation)

        self.infrastructure_objects_subscriber = self.create_subscription(DetectedObjects, \
                '/perception/object_recognition/detection/carom/objects', self.infrastructure_objects_df_converter, 10)
        
        self.transformed_objects_publisher = self.create_publisher(DetectedObjects, \
                '/perception/object_recognition/detection/carom_frame_transformed/objects', 10)
        
        self.entry_columns = ['existence_probability', 'source', 'object_class', 'x', 'y', 'z', 'o_x', 'o_y', 'o_z', 'o_w', 'd_x', 'd_y', 'd_z']
        self.infrastructure_objects_df = pd.DataFrame(columns=self.entry_columns)

        self.x_tf = 0
        self.y_tf = 0
        self.z_tf = 0
        self.roll_tf = 0
        self.pitch_tf = 0
        self.yaw_tf = 0

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
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return [x, y, z, w]
    
    def frame_transformation(self):
        try:
            t = self.tf_buffer.lookup_transform('base_link', 'camera_infrastructure_local_frame', rclpy.time.Time())
            self.yaw_tf = self.euler_from_quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)
            self.x_tf, self.y_tf, self.z_tf = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z 
            #print('yaw_tf', self.yaw_tf)
        except TransformException as ex:
            self.get_logger().info(f'Could not transform "base_link" to "camera_infrastructure_local_frame": {ex}')
            return 0
        
    def transform_camera_frame_x_to_base_frame_x(self, x, y, z, tx, yaw_angle):
        x_ = x*math.cos(yaw_angle) - y*math.sin(yaw_angle) + tx
		#print('x:',x, 'yaw_angle:', yaw_angle, 'x_', x_)
        return x_
    
    def transform_camera_frame_y_to_base_frame_y(self, x, y, z, ty, yaw_angle):
        y_ = x*math.sin(yaw_angle) + y*math.cos(yaw_angle) + ty
        return y_
    
    def transform_camera_frame_z_to_base_frame_z(self, x, y, z, tz, yaw_angle):
        z_ = z + tz
        return z_
    
    def objects_publisher(self):
        #self.start_time = time.time()
        publish_objects_msg = DetectedObjects()
        publish_objects_msg.header =  self.header_info
        publish_objects_msg.header.frame_id = "base_link"
        objects = [0 for o in range(len(self.infrastructure_objects_df))]
        classification = [0]
        for n in range(len(self.infrastructure_objects_df)):
            objects[n] = DetectedObject()
            classification[0] = ObjectClassification()
            objects[n].existence_probability = self.infrastructure_objects_df.iloc[n]['existence_probability']
            classification[0].label = int(self.infrastructure_objects_df.iloc[n]['object_class'])
            objects[n].classification = list(classification)
            objects[n].kinematics.pose_with_covariance.pose.position.x = self.infrastructure_objects_df.iloc[n]['x']
            objects[n].kinematics.pose_with_covariance.pose.position.y = self.infrastructure_objects_df.iloc[n]['y']
            objects[n].kinematics.pose_with_covariance.pose.position.z = self.infrastructure_objects_df.iloc[n]['z']
            objects[n].kinematics.pose_with_covariance.pose.orientation.x = self.infrastructure_objects_df.iloc[n]['o_x']
            objects[n].kinematics.pose_with_covariance.pose.orientation.y = self.infrastructure_objects_df.iloc[n]['o_y']
            objects[n].kinematics.pose_with_covariance.pose.orientation.z = self.infrastructure_objects_df.iloc[n]['o_z']
            objects[n].kinematics.pose_with_covariance.pose.orientation.w = self.infrastructure_objects_df.iloc[n]['o_w']
            objects[n].kinematics.orientation_availability = 1
            objects[n].shape.dimensions.x = self.infrastructure_objects_df.iloc[n]['d_x']
            objects[n].shape.dimensions.y = self.infrastructure_objects_df.iloc[n]['d_y']
            objects[n].shape.dimensions.z = self.infrastructure_objects_df.iloc[n]['d_z']
        
        publish_objects_msg.objects = objects

        #time_difference = time.time() - self.start_time
        #print("Time taken for message format conversion (For publish): ", time_difference*1000)
        
        self.transformed_objects_publisher.publish(publish_objects_msg)
        

        #time_difference = time.time() - self.start_time
        #print("Time taken for one iteration of frame transformation: ", time_difference*1000)
    
    def infrastructure_objects_frame_transformer(self):
        if not(self.infrastructure_objects_df.empty):
			#print(self.merged_objects_df)
			#x_, y_, z_ = self.infrastructure_objects_df['x'], self.infrastructure_objects_df['y'], self.infrastructure_objects_df['z']

            # start time of frame_transformation process alone
            #self.start_time = time.time()
            
            fix_x_tf_for_this_iteration = self.x_tf
            fix_y_tf_for_this_iteration = self.y_tf
            fix_z_tf_for_this_iteration = self.z_tf
            fix_yaw_tf_for_this_iteration = self.yaw_tf

            #self.start_time = time.time()

            self.infrastructure_objects_df['x_new'] = \
                self.infrastructure_objects_df.apply(lambda row: self.transform_camera_frame_x_to_base_frame_x(row['x'],row['y'],row['z'], fix_x_tf_for_this_iteration, fix_yaw_tf_for_this_iteration), axis=1)
            self.infrastructure_objects_df['y_new'] = \
                self.infrastructure_objects_df.apply(lambda row: self.transform_camera_frame_y_to_base_frame_y(row['x'],row['y'],row['z'], fix_y_tf_for_this_iteration, fix_yaw_tf_for_this_iteration), axis=1)
            self.infrastructure_objects_df['z_new'] = \
                self.infrastructure_objects_df.apply(lambda row: self.transform_camera_frame_z_to_base_frame_z(row['x'],row['y'],row['z'], fix_z_tf_for_this_iteration, fix_yaw_tf_for_this_iteration), axis=1)
            
            #time_difference = time.time() - self.start_time
            #print("Time taken for complete frame transformation process alone (x, y, z transformation alone): ", time_difference*1000)

            self.infrastructure_objects_df['x'] = self.infrastructure_objects_df['x_new']
            self.infrastructure_objects_df['y'] = self.infrastructure_objects_df['y_new']
            self.infrastructure_objects_df['z'] = self.infrastructure_objects_df['z_new']

            #self.start_time = time.time()

            self.infrastructure_objects_df['vehicle_angle'] = self.infrastructure_objects_df.apply(lambda row: self.euler_from_quaternion(row['o_x'], row['o_y'], row['o_z'], row['o_w']), axis=1)
            transformed_quaternion = self.infrastructure_objects_df.apply(lambda row: self.euler_to_quaternion(0, 0, row['vehicle_angle']+self.yaw_tf), axis=1)
            transformed_quaternion_x_array, transformed_quaternion_y_array,transformed_quaternion_z_array,transformed_quaternion_w_array = [], [], [], []
            for m in range(len(transformed_quaternion)):
                transformed_quaternion_x_array.append(transformed_quaternion[m][0])
                transformed_quaternion_y_array.append(transformed_quaternion[m][1])
                transformed_quaternion_z_array.append(transformed_quaternion[m][2])
                transformed_quaternion_w_array.append(transformed_quaternion[m][3])

			#print("quaternion_array: ", transformed_quaternion_x_array, transformed_quaternion_y_array,transformed_quaternion_z_array,transformed_quaternion_w_array)
            transformed_quaternion_x = pd.Series(transformed_quaternion_x_array)
            transformed_quaternion_y = pd.Series(transformed_quaternion_y_array)
            transformed_quaternion_z = pd.Series(transformed_quaternion_z_array)
            transformed_quaternion_w = pd.Series(transformed_quaternion_w_array)
            
            self.infrastructure_objects_df['o_x'], self.infrastructure_objects_df['o_y'], self.infrastructure_objects_df['o_z'], self.infrastructure_objects_df['o_w'] = transformed_quaternion_x, transformed_quaternion_y, transformed_quaternion_z, transformed_quaternion_w

            #time_difference = time.time() - self.start_time
            #print("Time taken for complete frame transformation process alone (orientation transform alone): ", time_difference*1000)

			#print("transformed quaternion: ", transformed_quaternion_x, transformed_quaternion_y, transformed_quaternion_z, transformed_quaternion_w)
            self.infrastructure_objects_df = self.infrastructure_objects_df.drop(['x_new', 'y_new', 'z_new', 'vehicle_angle'], axis='columns')

            #time_difference = time.time() - self.start_time
            #print("Time taken for frame transformation process alone: ", time_difference*1000)

            self.objects_publisher()
            self.infrastructure_objects_df = pd.DataFrame(columns=self.entry_columns)
    
    def infrastructure_objects_df_converter(self, infrastructure_detection_msg):
		#self.get_logger().info('I heard: "%s"' %infrastructure_detection_msg.objects[0].classification)
		#self.get_logger().info('Number of objects: "%s"' %len(infrastructure_detection_msg.objects))

        # start time of the frame transformation process
        self.start_time = time.time()

        self.header_info = infrastructure_detection_msg.header
        for j in range(len(infrastructure_detection_msg.objects)):
            infrastructure_object_info_autoware_format = infrastructure_detection_msg.objects[j]
            infrastructure_object_info = [infrastructure_object_info_autoware_format.existence_probability, 1,
							infrastructure_object_info_autoware_format.classification[0].label, 
							infrastructure_object_info_autoware_format.kinematics.pose_with_covariance.pose.position.x,
							infrastructure_object_info_autoware_format.kinematics.pose_with_covariance.pose.position.y,
							infrastructure_object_info_autoware_format.kinematics.pose_with_covariance.pose.position.z,
							infrastructure_object_info_autoware_format.kinematics.pose_with_covariance.pose.orientation.x,
							infrastructure_object_info_autoware_format.kinematics.pose_with_covariance.pose.orientation.y,
							infrastructure_object_info_autoware_format.kinematics.pose_with_covariance.pose.orientation.z,
							infrastructure_object_info_autoware_format.kinematics.pose_with_covariance.pose.orientation.w,
							infrastructure_object_info_autoware_format.shape.dimensions.x,
							infrastructure_object_info_autoware_format.shape.dimensions.y,
							infrastructure_object_info_autoware_format.shape.dimensions.z]
            infrastructure_object_info_dict = dict(zip(self.entry_columns, list(infrastructure_object_info)))
            self.infrastructure_objects_df = self.infrastructure_objects_df.append(infrastructure_object_info_dict, ignore_index=True)
			#print(self.infrastructure_objects_df.head())
        
        time_difference = time.time() - self.start_time
        print("Time taken for message format conversion (during subscribtion): ", time_difference*1000)

        self.infrastructure_objects_frame_transformer()  

          


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()