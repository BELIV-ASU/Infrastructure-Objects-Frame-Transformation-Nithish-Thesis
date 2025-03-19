import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time

from tf2_ros import TransformBroadcaster

class BaseToCameraTFBroadcaster(Node):
    def __init__(self):
        super().__init__('base_to_camera_frame_tf_broadcaster')

        self.odometry_subscription = self.create_subscription(Odometry, \
                '/localization/kinematic_state', self.odometry_filter, 10)
        
        self.time_copy = Time()
        
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x_tf = -49.0
        self.y_tf = -5.0
        self.z_tf = 0.0
        self.ox_tf = 0.0
        self.oy_tf = 0.0
        self.oz_tf = 0.0
        self.ow_tf = 1.0

        self.timer = self.create_timer(0.01, self.on_timer)

    def odometry_filter(self, msg):
        self.time_copy = msg.header.stamp

    def on_timer(self):
        t1 = TransformStamped()
        t2 = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t1.header.stamp = self.time_copy
        t1.header.frame_id = 'camera_infrastructure_local_frame'
        t1.child_frame_id = 'base_link'

        t1.transform.translation.x = self.x_tf
        t1.transform.translation.y = self.y_tf
        t1.transform.translation.z = self.z_tf

        t1.transform.rotation.x = self.ox_tf
        t1.transform.rotation.y = self.oy_tf
        t1.transform.rotation.z = self.oz_tf
        t1.transform.rotation.w = self.ow_tf

        self.x_tf = self.x_tf + 0.05
        #self.y_tf = self.x_tf + 0.05

        t2.header.stamp = self.time_copy
        t2.header.frame_id = 'map'
        t2.child_frame_id = 'camera_infrastructure_local_frame'

        t2.transform.translation.x = -47.05
        t2.transform.translation.y = 0.1
        t2.transform.translation.z = 0.0

        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = -0.706825
        t2.transform.rotation.w = 0.707388

        # Send the transformation
        self.tf_broadcaster.sendTransform(t1)
        self.tf_broadcaster.sendTransform(t2)

def main():
    rclpy.init()
    node = BaseToCameraTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()