'''This is a ROS node designed to log any data coming from the otter to a csv. It takes everything output by the otter_publisher node
and writes it to a csv'''

import rclpy
from rclpy.node import Node
import message_filters as msg_fltrs
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Vector3Stamped
from otter_msgs.msg import COGSOG, ControllerErrors
import casadi as cs
import numpy as np
import navpy as nav
import scipy.ndimage
import math
import datetime


class NMPC(Node):

    def __init__(self):
        super().__init__('node')
        # declaring te wpt_file location as a paramenter
        self.declare_parameter('wpt_file', None)
        # define the subscribers        
        self.gps_subscriber = msg_fltrs.Subscriber(self, NavSatFix, 'otter_gps')
        self.cogsog_subscriber = msg_fltrs.Subscriber(self, COGSOG, 'otter_cogsog')
        self.imu_subscriber = msg_fltrs.Subscriber(self, Imu, 'otter_imu')

        # link them together in the approximate time synchronizer, the 'slop' value is the interval of time that
        # commands must be within for them to be synchronized
        self.ts = msg_fltrs.ApproximateTimeSynchronizer([self.gps_subscriber, self.cogsog_subscriber, self.imu_subscriber], 10, slop=1)
        self.ts.registerCallback(self.callback)

        # Define the publisher for the control inputs
        self.ctrl_pub = self.create_publisher(WrenchStamped, 'control_cmds', 10)
        self.ctrl_errs_pub = self.create_publisher(ControllerErrors, 'controller_errs', 10)

        ##################### Get Wpts ###############################
        self.wpt_file = self.get_parameter("wpt_file").value
        # read in wpt data from the correct location
        self.wpts, self.speed_d, R_switch = self.read_wpt_data(self.wpt_file)
        self.num_wpts = self.wpts.shape[1]
        self.wpt_window_size = 40  # how big to make the wpt window
        self.get_logger().info(f'wpts:{self.num_wpts}')

        ############## Parmeter and Model Definition ###############################
        # Define time step and prediction horizon

        self.T = 0.1  # time step
        self.H = 40  # prediction horizon

        # Define some parameters of our robot
        self.xy_max = cs.inf
        self.psi_max = cs.inf
        self.u_max = 1.5
        self.v_max = 1.5

        self.r_max = cs.pi / 6
        self.tauX_max = 292  # 33lbs from datasheet to 146 N x2 motors is 292 N max.
        self.tauN_max = 292

        self.xy_min = -self.xy_max
        self.psi_min = -self.psi_max
        self.u_min = -self.u_max
        self.v_min = -self.v_max
        self.r_min = -self.r_max
        self.tauX_min = -self.tauX_max
        self.tauN_min = -self.tauN_max

        self.m1 = 60.5
        self.m2 = 137.5
        self.m3 = 45.265
        self.d1 = 77.554
        self.d2 = 0
        self.d3 = 45.265

        # Define some symbols for states and control and our vehicle model in casadi
        x = cs.SX.sym('x')
        y = cs.SX.sym('y')
        psi = cs.SX.sym('psi')
        u = cs.SX.sym('u')
        v = cs.SX.sym('v')
        r = cs.SX.sym('r')
        states = cs.vertcat(x, y, psi, u, v, r)
        self.n_states = states.shape[0]

        tauX = cs.SX.sym('tauX')
        tauN = cs.SX.sym('tauN')
        controls = cs.vertcat(tauX, tauN)
        self.n_controls = controls.shape[0]

        rhs = cs.vertcat(
            u * cs.cos(psi) - v * cs.sin(psi),
            u * cs.sin(psi) + v * cs.cos(psi),
            r,
            (self.m2 / self.m1) * v * r - (self.d1 / self.m1) * u + (tauX / self.m1),
            -(self.m1 / self.m2) * u * r - (self.d2 / self.m2) * v,
            ((self.m1 - self.m2) / self.m3) * u * v - (self.d3 / self.m3) * r + (tauN / self.m3)
        )

        self.f = cs.Function('f', [states, controls], [rhs])

        ################# Optimization Problem Definition ####################

        # Define variables and parameters for the optimization algorithm
        U = cs.MX.sym('U', self.n_controls, self.H)  # controls to optimize over
        P = cs.MX.sym('P', self.n_states + self.wpt_window_size * 2 * 2 + 1)  # parameters, first 3 elements are initial state and last 3 are desired pose
        X = cs.MX.sym('X', self.n_states, self.H + 1)  # state values

        # Define weight matrices
        self.Q = np.diag([2000, 1500, 5000])
        self.R = np.diag([1, 0.7])

        # Define the cost function
        self.cost = 0
        # Define a new function which integrates the model to give the series of states given a series of inputs
        g = X[:, 0] - P[0:self.n_states]
        sim_wpt_ctr = P[-1]  # This is the actual closest wpt to the real vehicle
        # this is the calculated window of wpts that the optimization will be performed within
        sim_wpt_window = cs.reshape(P[-(self.wpt_window_size * 2 * 2 + 1):-1], 2, 2 * self.wpt_window_size)
        # desired surge
        self.surge_d = 1.5
        # setup the optimization, get error vectors, calculate cost, get the next state, and save the conditions in g
        for k in range(self.H):
            # get
            err, sim_wpt_ctr = self.get_error_state(X[:, k], self.surge_d, sim_wpt_window)
            self.cost = self.cost + err.T @ self.Q @ err + U[:, k].T @ self.R @ U[:, k]
            k1 = self.f(X[:, k], U[:, k])
            k2 = self.f(X[:, k] + self.T / 2 * k1, U[:, k])
            k3 = self.f(X[:, k] + self.T / 2 * k2, U[:, k])
            k4 = self.f(X[:, k] + self.T * k3, U[:, k])
            next_state = X[:, k] + self.T / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            g = cs.vertcat(g, (next_state - X[:, k + 1]))

        # reshape the input values to be a single list as this is what is required. Also define the nlp_prob
        opt_variables = cs.horzcat(cs.reshape(X, 1, self.n_states * (self.H + 1)), cs.reshape(U, 1, self.n_controls * self.H))
        self.nlp_prob = {'f': self.cost, 'x': opt_variables, 'g': g, 'p': P}

        # Define optimization options
        self.options = {'ipopt.max_iter': 2000, 'ipopt.print_level': 0, 'print_time': 0, 'ipopt.acceptable_tol': 1e-8,
                   'ipopt.acceptable_obj_change_tol': 1e-6}
        self.solver = cs.nlpsol('solver', 'ipopt', self.nlp_prob, self.options)

        # Define lower and upper bounds on state positions
        self.lbg = 0
        self.ubg = 0

        # Define lower and upper bounds on inputs/optimization variables. Since the inputs are now [v, omega, v, omega,...]
        # We need to define them as alternating values
        self.lbx = np.zeros((self.n_states * (self.H + 1) + self.n_controls * self.H))
        self.ubx = np.zeros((self.n_states * (self.H + 1) + self.n_controls * self.H))

        self.lbx[0:self.n_states * (self.H + 1):self.n_states] = self.xy_min
        self.lbx[1:self.n_states * (self.H + 1):self.n_states] = self.xy_min
        self.lbx[2:self.n_states * (self.H + 1):self.n_states] = self.psi_min
        self.lbx[3:self.n_states * (self.H + 1):self.n_states] = self.u_min
        self.lbx[4:self.n_states * (self.H + 1):self.n_states] = self.v_min
        self.lbx[5:self.n_states * (self.H + 1):self.n_states] = self.r_min
        self.lbx[self.n_states * (self.H + 1)::self.n_controls] = self.tauX_min
        self.lbx[self.n_states * (self.H + 1) + 1::self.n_controls] = self.tauN_min

        self.ubx[0:self.n_states * (self.H + 1):self.n_states] = self.xy_max
        self.ubx[1:self.n_states * (self.H + 1):self.n_states] = self.xy_max
        self.ubx[2:self.n_states * (self.H + 1):self.n_states] = self.psi_max
        self.ubx[3:self.n_states * (self.H + 1):self.n_states] = self.u_max
        self.ubx[4:self.n_states * (self.H + 1):self.n_states] = self.v_max
        self.ubx[5:self.n_states * (self.H + 1):self.n_states] = self.r_max
        self.ubx[self.n_states * (self.H + 1)::self.n_controls] = 0.8*self.tauX_max
        self.ubx[self.n_states * (self.H + 1) + 1::self.n_controls] = 0.8*self.tauN_max

        # These are the initial inputs over the horizon
        self.u0 = np.zeros((self.n_controls, self.H))
        self.X0 = cs.reshape(cs.repmat([0,0,0,0,0,0], self.H + 1), self.n_states, self.H + 1)

        # Log that node is initialized
        self.get_logger().info('nmpc_controller node initialized.')

        self.first_run = True


    # This is for converting a state value read from the otter system from lla to ned coordinates using the same origin
    # as the waypoint path
    def state_2_ned(self, lat, lon):
        ned_point = nav.lla2ned(lat, lon, 0, self.origin_lat, self.origin_lon, self.origin_alt, latlon_unit='deg',
                               alt_unit='m',
                               model='wgs84')
        return ned_point

    def read_wpt_data(self, wpt_file):
        # read in the data and create arrays of the wpt lat on lon locations
        wpts_data = np.genfromtxt(wpt_file, delimiter='\t')
        wpts_lat = wpts_data[:, 0]
        wpts_lon = wpts_data[:, 1]
        alts = np.zeros(
            wpts_lon.shape)  # the altitude isn't provided, so I just create a zero matrix of the same length
        # convert the info to local NED frame data were the 0th wpt is the origin
        self.origin_lat = wpts_lat[0]
        self.origin_lon = wpts_lon[0]
        self.origin_alt = 0
        self.get_logger().info(f'origin: {self.origin_lat}, {self.origin_lon}, {self.origin_alt}')
        ned_data = nav.lla2ned(wpts_lat, wpts_lon, alts, wpts_lat[0], wpts_lon[0], 0, latlon_unit='deg',
                               alt_unit='m',
                               model='wgs84')
        # here we interpolate the waypoints to a higher granularity
        interp = self.interp_waypoints(ned_data)
        # get the desired speed data at each point, should be constant across the trajectory
        speed = wpts_data[0, 3]
        R_switch = wpts_data[0, 4]  # get the switching circle radius just in case it's needed

        return interp, speed, R_switch

    # This is the function that interpolates the waypoints
    def interp_waypoints(self, wpts, loop=False):
        x_vals = []
        y_vals = []
        # loop through the waypoints and interpolate the distance for each set of waypoints
        for i, pt in enumerate(wpts[:, 0]):
            if i != (len(wpts[:, 0]) - 1):
                dist_bw = np.sqrt((wpts[i + 1, 0] - wpts[i, 0]) ** 2 + (wpts[i + 1, 1] - wpts[i, 1]) ** 2)
                x_pts = wpts[i:i + 2, 0]
                y_pts = wpts[i:i + 2, 1]
            elif loop == True:
                dist_bw = np.sqrt((wpts[0, 0] - wpts[i, 0]) ** 2 + (wpts[0, 1] - wpts[i, 1]) ** 2)
                x_pts = [wpts[i, 0], wpts[0, 0]]
                y_pts = [wpts[i, 1], wpts[0, 1]]
            num_pts = np.ceil(dist_bw * 4)
            zoom = np.round(num_pts / 2)
            x_interped = scipy.ndimage.zoom(x_pts, zoom)
            y_interped = scipy.ndimage.zoom(y_pts, zoom)
            x_vals += x_interped.tolist()
            y_vals += y_interped.tolist()
        #end=int(len(x_vals)/2)
        #interp_wpts = np.array([x_vals[:end], y_vals[:end]])
        interp_wpts = np.array([x_vals,y_vals])
        return interp_wpts

    # This function gets us a smaller window of wpts from the larger set so that we can work with fewer pts in the
    # optimization, the window is +-window_size points around the wpt_ctr provided
    def get_wpt_window(self, wpt_ctr, wpts, window_size=10):
        # Here we convert the wpt_ctr variable to an integer from a DM which is what the get_closest_wpt function returns it
        # as since it is the same function used in the optimization
        wpt_ctr = int(wpt_ctr.elements()[0])
        # These if statements are for if our window goes over the origin, we need to handle adjusting the ranges of pts to
        # choose

        # if by looking backwards we pass the 0th point
        if 0 <= wpt_ctr - window_size:

            # if by looking forwards we pass the 0th point
            if wpt_ctr + window_size < wpts.shape[1]:
                wpt_window = wpts[:, wpt_ctr - window_size:wpt_ctr + window_size]
            else:
                ctr_diff = window_size - (wpts.shape[1] - 1 - wpt_ctr) - 1
                wpt_window = np.concatenate((wpts[:, (wpt_ctr - window_size):], wpts[:, 0:ctr_diff]),
                                            axis=1)
        else:
            ctr_diff = window_size - wpt_ctr
            wpt_window = np.concatenate((wpts[:, -ctr_diff:], wpts[:, :(wpt_ctr + window_size)]),
                                        axis=1)

        return wpt_window

    # This function gets us the closest wpt to our current position and also returns the lateral error at the same time.
    # This function looks like it could be improved using argmin() or a similar function, however, I found this difficult
    # to do with symbolics and this is the most optimal solution I could think of.
    def get_closest_wpt(self, state, wpt_list):
        # get the euclidean distances to all waypoints
        distances = cs.hypot((wpt_list[0, :] - state[0]), (wpt_list[1, :] - state[1]))
        # get the minimum distance
        min_dist = cs.mmin(distances)
        # get the wpt that corresponds to the minimum distance
        closest_wpt = cs.DM(0)
        for i in range(1, distances.shape[0]):
            closest_wpt = cs.if_else(cs.lt(min_dist, distances[i]), closest_wpt, i)
        return closest_wpt, min_dist

    # get the current course error
    def get_heading_err(self, wpt_ctr, wpts, state):
        # Since this is in the optimization, turn the wpt_window into an MX so it can be indexed using symbolics
        wpts = cs.MX(wpts)
        # get the desired orientation angle by finding the vector between the closest wpt and the next wpt after the closest
        # If the closest wpt is at the end of the list use the angle between the end wpt and the second to last wpt.
        pi_p = cs.if_else(cs.ge(wpt_ctr, wpts.shape[1] - 1),
                          cs.atan2(wpts[1, wpt_ctr] - wpts[1, wpt_ctr - 1],
                                   wpts[0, wpt_ctr] - wpts[0, wpt_ctr - 1]),
                          cs.atan2(wpts[1, wpt_ctr + 1] - wpts[1, wpt_ctr],
                                   wpts[0, wpt_ctr + 1] - wpts[0, wpt_ctr]))
        # This is an example of how to print the values of the symbolics mid run for debugging purposes. Just put
        # numbers into the function can uncomment these functions and run the debugger or print the values
        # test1 = cs.evalf(pi_p)
        # test3 = cs.evalf(yaw - pi_p)
        # Calculate the heading error by subtracting the values and then ensuring it stays between -pi and pi
        course_err = cs.remainder(pi_p - state[2], math.tau)
        return course_err

    # get the collective error state
    def get_error_state(self, state, surge_d, wpt_window):
        # find the closest wpt to current position and get the corresponding lateral error
        closest_wpt, lateral_error = self.get_closest_wpt(state, wpt_window)
        # find the heading error based on the closest wpt
        heading_err = self.get_heading_err(closest_wpt, wpt_window, state)
        # # Get the surge error
        surge_err = surge_d - state[3]
        # concatenate them before you return
        err = cs.vertcat(lateral_error, heading_err, surge_err)
        return err, closest_wpt
    
    # function for rotating a 2d vector to the body frame
    def rotate_2_body(self,u,v,psi):
        c = np.cos(psi)
        s = np.sin(psi)
        rot_array = np.array([[c, s], [-1*s,c]])
        vect = np.matmul(rot_array, np.asarray([u,v]))
        return vect
    
    # function for rconverting from speed in knots to m/s
    def knots_2_mps(self,u):
        u = u*1.94384
        return u
    
    # This is the callback for gps and imu, write the data to the csv and screen as it comes in
    # The read_wpt_data function is set up to read the data from the csvs provided by the MR Otter VCS for the waypoint
    # paths it creates
    def callback(self, gps_msg, cogsog_msg, imu_msg):
        start = datetime.datetime.now()
        # get the state value from the msg
        x_val = gps_msg.latitude
        y_val = gps_msg.longitude
        psi_val = np.deg2rad(imu_msg.orientation.z)
        u_val = cogsog_msg.twist.linear.x
        v_val = cogsog_msg.twist.linear.y
        r_val = np.deg2rad(imu_msg.angular_velocity.z)
        ned = self.state_2_ned(lat=x_val, lon=y_val)
        speed_vect = self.knots_2_mps(self.rotate_2_body(u_val, v_val, psi_val)) 
        x0 = [ned[0], ned[1], psi_val, speed_vect[0], speed_vect[1], r_val]
        self.get_logger().info(f'state:{x0}')

        get_time = datetime.datetime.now()

        # initialize wpt counter to get the closest wpt to x0 and determine the wpt_window around it
        if self.first_run:
            self.first_run = False
            self.wpt_ctr= self.get_closest_wpt(x0, self.wpts)[0]
            self.get_logger().info(f'wpt_ctr:{self.wpt_ctr}')
            self.wpt_window = self.get_wpt_window(self.wpt_ctr, self.wpts, window_size=self.wpt_window_size)
        err_state, self.wpt_offset = self.get_error_state(x0, self.surge_d, self.wpt_window)
        err_state = cs.evalf(err_state)
        self.wpt_ctr += self.wpt_offset - self.wpt_window_size-1
        self.wpt_window = self.get_wpt_window(self.wpt_ctr, self.wpts, window_size=self.wpt_window_size)
        # initialize wpt counter to get the closest wpt to x0 and determine the wpt_window around it
        # self.wpt_ctr = self.get_closest_wpt(x0, self.wpts)[0]
        # self.wpt_window = self.get_wpt_window(self.wpt_ctr, self.wpts, window_size=self.wpt_window_size)
        # err_state = self.get_error_state(x0, self.surge_d, self.wpt_window)[0]
        # err_state = cs.evalf(err_state)

        window_time = datetime.datetime.now()

        # set up the parameters and optimization variables to be the appropriate shape.
        opt_var0 = cs.horzcat(cs.reshape(self.X0, 1, self.n_states * (self.H + 1)), cs.reshape(self.u0, 1, self.n_controls * self.H))
        # we go the window_size fwd and back and there are 2 pts per location therefore we need to multiply by 4
        wpt_window = cs.reshape(self.wpt_window, 4 * self.wpt_window_size, 1)
        parameters = cs.vertcat(x0, wpt_window, self.wpt_ctr)
        # run the optimization
        sol = self.solver(x0=opt_var0, lbx=self.lbx, ubx=self.ubx, lbg=self.lbg, ubg=self.ubg, p=parameters)


        # reshape the control vals and save everything so we can plot and look at data later
        ctrl_pred = cs.reshape(sol['x'][self.n_states * (self.H + 1):], self.n_controls, self.H)
        state_pred = cs.reshape(sol['x'][:self.n_states * (self.H + 1)], self.n_states, self.H + 1)

        solve_time = datetime.datetime.now()

        # warming up the inputs
        self.u0[:, 0:-1] = ctrl_pred[:, 1:]
        self.u0[:, -1, np.newaxis] = ctrl_pred[:, -1]
        self.X0[:, 0:-1] = state_pred[:, 1:]
        self.X0[:, -1] = state_pred[:, -1]

        self.get_logger().info(f'thrust:{ctrl_pred[0,0]}, moment:{ctrl_pred[1,0]}')

        # Setup the ROS2 command for the torque and publish it
        ctrl_cmd = WrenchStamped()
        ctrl_cmd.header.stamp = self.get_clock().now().to_msg()
        ctrl_cmd.wrench.force.x = float(ctrl_pred[0,0]/self.tauX_max)
        ctrl_cmd.wrench.force.y = float(0)
        ctrl_cmd.wrench.torque.z = float(ctrl_pred[1,0]/self.tauN_max)

        self.ctrl_pub.publish(ctrl_cmd)

        # Setup the ROS2 command for the torque and publish it
        err_cmd = ControllerErrors()
        err_cmd.header.stamp = self.get_clock().now().to_msg()
        err_cmd.wpt_ctr = float(self.wpt_ctr)
        err_cmd.lateral_error = float(err_state[0])
        err_cmd.heading_error = float(err_state[1])
        err_cmd.speed_error = float(err_state[2])

        self.ctrl_errs_pub.publish(err_cmd)

        pub_time = datetime.datetime.now()

        delta_t = datetime.datetime.now()-start
        self.get_logger().info(f'total:{delta_t}, get:{get_time-start}, window:{window_time-get_time}, solve:{solve_time-window_time}, pub_time: {pub_time-solve_time}')

# Run the node
def main(args=None):
    rclpy.init(args=args)
    controller = NMPC()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
