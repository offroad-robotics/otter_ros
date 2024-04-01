import rclpy
from rclpy.node import Node
import ros2_message_filters as msg_fltrs
import navpy as nav
import math
import numpy as np
import pynmeagps as nmea
import socket
from geometry_msgs.msg import Vector3Stamped, PointStamped, Vector3, Point
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class LOSHeadController(Node):

    def __init__(self, wpts_file, tau_x = 0.5, look_ahead=5, m=41.4, wn=1.2, zeta=0.8):
        super().__init__('node')
        self.odom_subscriber = self.create_subscription(Odometry, 'otter_odom', self.callback, 10)

        self.publisher = self.create_publisher(Vector3, 'control_cmds', 10)

        self.wpts, self.speed_d, self.R_switch = self.read_wpt_data(wpts_file)
        self.num_wpts = self.wpts.shape[0] - 1
        self.lat_orig = self.wpts[0, 0]
        self.lon_orig = self.wpts[0, 1]
        self.wpt_ctr = 0

        self.look_ahead = look_ahead
        self.tau_x = tau_x

        # Model parameters for the otter (may need to change these)
        self.m = m  # moment of inertia in yaw including added mass
        T = 1
        K = T / self.m
        d = 1 / K
        k = 0
        self.wn = wn
        self.zeta = zeta

        # PID gains based on pole placement
        self.Kp = m * wn ** 2 - k
        self.Kd = m * 2 * zeta * wn - d
        self.Ki = (wn / 10) * self.Kp

        self.e_int = 0


    def ssa(self, angle):
        """
        angle = ssa(angle) returns the smallest-signed angle in [ -pi, pi )
        """
        angle = (angle + math.pi) % (2 * math.pi) - math.pi

        return angle

    def read_wpt_data(self, wpt_file):
        wpts_data = np.genfromtxt(wpt_file, delimiter='\t')
        wpts_lat = wpts_data[:, 0]
        wpts_lon = wpts_data[:, 1]
        alts = np.zeros(wpts_lon.shape)
        ned_data = nav.lla2ned(wpts_lat, wpts_lon, alts, wpts_lat[0], wpts_lon[0], 0, latlon_unit='deg', alt_unit='m',
                               model='wgs84')
        speed = wpts_data[0,3]
        R_switch = wpts_data[0,4]

        return ned_data, speed, R_switch

    def LOS_heading(self, x, y, crab_angle, speed=2):
        # info on strategy on p.349
        # Compute the desired course angle w.r.t. North
        wpts_x = self.wpts[:,0]
        wpts_y = self.wpts[:,1]
        pi_h = np.arctan2(wpts_y[(self.wpt_ctr + 1) % self.num_wpts] - wpts_y[self.wpt_ctr],
                          wpts_x[(self.wpt_ctr + 1) % self.num_wpts] - wpts_x[self.wpt_ctr])
        dist_bw_wps = np.sqrt((wpts_x[(self.wpt_ctr + 1) % self.num_wpts] - wpts_x[self.wpt_ctr]) ** 2 + (
                wpts_y[(self.wpt_ctr + 1) % self.num_wpts] - wpts_y[self.wpt_ctr]) ** 2)

        # along-track and cross-track errors (x_e, y_e)
        x_e = (x - wpts_x[self.wpt_ctr]) * np.cos(pi_h) + (y - wpts_y[self.wpt_ctr]) * np.sin(pi_h)
        y_e = -(x - wpts_x[self.wpt_ctr]) * np.sin(pi_h) + (y - wpts_y[self.wpt_ctr]) * np.cos(pi_h)

        # if the next waypoint satisfy the switching criterion, k = k + 1
        # print(self.wpt_ctr)
        if (dist_bw_wps - x_e < self.R_switch) and (self.wpt_ctr < wpts_x.shape[0] - 1):
            self.wpt_ctr = (self.wpt_ctr + 1) % self.num_wpts

        # LOS guidance law
        psi_d = pi_h - np.arctan(y_e / self.look_ahead) - crab_angle  # p.363

        # desired course rate: omega_chi_d[]
        dy_e = -speed * y_e / np.sqrt(self.look_ahead ** 2 + y_e ** 2)  # d/dt y_e p.356
        r_d = -(1 / self.look_ahead) * dy_e / (1 + (y_e / self.look_ahead) ** 2)  # derivative of chi_d equation
        return psi_d, r_d

    def heading_controller(self, psi, r, psi_d, r_d, sampleTime):
        e_psi = self.ssa(psi - psi_d)  # yaw angle tracking error
        e_r = r - r_d  # yaw rate tracking error

        tau_X = self.tau_x

        # PID control law
        tau_N = -self.Kp * e_psi - self.Kd * e_r - self.Ki * self.e_int

        # Integral error, Euler's method
        self.e_int += sampleTime * e_psi

        # [n1, n2] = self.controlAllocation(tau_X, tau_N)
        # u_control = np.array([n1, n2], float)

        return tau_X, tau_N

    def callback(self, odom_msg):
        ned_pos = nav.lla2ned(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z, self.lat_orig, self.lon_orig, 0, latlon_unit='deg', alt_unit='m',
                                 model='wgs84')
        
        x = ned_pos[0]
        y = ned_pos[1]
        psi = odom_msg.pose.pose.orientation.z  # yaw angle
        r = odom_msg.twist.twist.angular.z  # yaw rate
        course = odom_msg.twist.twist.linear.y
        speed = odom_msg.twist.twist.linear.x

        crab_angle = self.ssa(course - psi)

        [psi_d, r_d] = self.LOS_heading(x, y, crab_angle, speed)

        tau_X, tau_N = self.heading_controller(psi,r,psi_d,r_d, sampleTime=0.002)

        ctrl_msg = Vector3()
        ctrl_msg.x = tau_X
        ctrl_msg.y = 0
        ctrl_msg.z = tau_N

        self.publisher.publish(ctrl_msg)


def main(args=None, wpts_file='Pier_circle_50rad.csv'):
    rclpy.init(args=args)

    otter_controller = LOSHeadController(wpts_file)

    rclpy.spin(otter_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    otter_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(wpts_file='20m-rad_circle.csv')