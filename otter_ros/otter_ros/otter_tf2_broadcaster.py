'''
OtterROS tf2 broadcaster

Transforms the IMU data from the otter into a tf2 frame.
This is a dynamic broadcaster, so it will update the frame as the IMU data updates.

Authors:
Dan Jenkins 2023
Thomas Sears 2024
'''
import rclpy
from rclpy.node import Node

# import geodesy
# from geodesy.utm import *

import numpy as np
from scipy.spatial.transform import Rotation as R

from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry


def quaternion_from_euler(ai, aj, ak):
    # convert to radians
    ai *= np.pi/180.0
    aj *= np.pi/180.0
    ak *= np.pi/180.0

    # convert to quaternions
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = np.cos(ai)
    si = np.sin(ai)
    cj = np.cos(aj)
    sj = np.sin(aj)
    ck = np.cos(ak)
    sk = np.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class TransformPublisher(Node):

    def __init__(self):
        super().__init__('otter_tf2_frame_publisher')

        self.ottername = self.declare_parameter(
            'ottername', 'otter').get_parameter_value().string_value

        # Subscriber for IMU data
        self.imu_subscriber = self.create_subscription(
            Imu, 'otter_imu', self.imu_callback, 10)
        self.imu_subscriber

        # Subscruber for GPS data
        # self.gps_subscriber = self.create_subscription(
        #     NavSatFix, 'otter_gps', self.gps_callback, 10)
        # self.gps_subscriber

        # Publishers for transforms
        self.tf_broadcaster = TransformBroadcaster(self)

        # self.t = TransformStamped()

        # self.t.header.frame_id = 'world'
        # self.t.child_frame_id = self.ottername

        # self.flag = 0
        # self.origin_north = 0
        # self.origin_east = 0
        # self.origin_alt = 0

    def imu_callback(self, msg):
        t = TransformStamped()

        # t.header.stamp = msg.header.stamp
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world_ned'
        t.child_frame_id = self.ottername

        # ignoring translation for now
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(
            msg.orientation.x, msg.orientation.y, msg.orientation.z)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    # def gps_callback(self, msg):

    #     global flag
    #     global Onorth
    #     global Oeast
    #     global Oalt
    #     global t
    #     t.header.stamp = msg.header.stamp
    #     t.header.frame_id = 'world'
    #     t.child_frame_id = 'base_link'

    #     # initialize the origin
    #     if flag == 0:
    #         flag = 1
    #         UTM = geodesy.utm.fromLatLong(
    #             msg.latitude, -msg.longitude, msg.altitude)
    #         Onorth = UTM.northing
    #         Oeast = UTM.easting
    #         Oalt = UTM.altitude

    #     # calculate UTM (meters) from Lat and Long, then get a distance from Origin. -ve long because we are west not east.
    #     else:
    #         UTM = geodesy.utm.fromLatLong(
    #             msg.latitude, -msg.longitude, msg.altitude)
    #         Cnorth = UTM.northing
    #         Ceast = UTM.easting
    #         Calt = UTM.altitude
    #         t.transform.translation.y = Cnorth - Onorth
    #         t.transform.translation.x = Ceast - Oeast
    #         t.transform.translation.z = Calt - Oalt

    #         self.tf_broadcaster.sendTransform(t)

# Run the node


def main(args=None):
    rclpy.init(args=args)

    node = TransformPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
