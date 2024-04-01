"""OtterROS Node - Otter Subscriber (otter_subscriber)

Send external commands to the Otter USV "OBS" from ROS topics.

**Requires the customized "pynmeagps" package included in the Otter ROS repository**

"""

# Import the necessary packages
# ROS2 packages
import rclpy
from rclpy.node import Node

# ROS2 interfaces
from std_msgs.msg import String  # flag for removal
from geometry_msgs.msg import WrenchStamped
from otter_msgs.msg import ExtCmdStation

# Third party python packages
import numpy as np
import socket

# Custom version of pynmeagps package
import pynmeagps as nmea


class OtterSubscriber(Node):

    def __init__(self):
        super().__init__('otter_subscriber')

        # Define the network settings where we need to send the external (ROS) commands
        # These are taken from the MR external interface documentation
        self.declare_parameter('port', 2009)
        self.declare_parameter('obs_ip_address', "192.168.53.2")

        # Create external command subscribers

        # Subscriber for MANUAL commands; these look at the control_cmds topic
        self.control_sub = self.create_subscription(
            WrenchStamped, 'control_cmds', self.send_cmd_manual, 10)

        # Subscriber for STATION KEEPING commands; these look at the station_keeping_cmds topic
        self.station_sub = self.create_subscription(
            ExtCmdStation, 'station_keeping_cmds', self.send_cmd_station, 10)

        self.stream = self.socket_connect()

        # Log that node is initialized
        self.get_logger().info('otter_subscriber node initialized.')

    # This creates the socket connection
    def socket_connect(self):
        stream = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        stream.connect((self.get_parameter('obs_ip_address').value,
                       self.get_parameter('port').value))
        return stream

    # This logs the message to display
    def display(self, msg):
        self.get_logger().info(f'I sent: {msg}')

    # Send command functions

    # This sends the manual command
    def send_cmd_manual(self, msg):
        # msg must be a WrenchStamped message (see ROS2 documentation)
        vec = np.array(
            [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.torque.z])

        # vec is a numpy.array of surge, sway, and torque inputs:
        # surge [-1,1], sway [-1,1], torque [-1,1]
        # Note: for Otter USV, sway should be left at zero!
        # Examples:
        #   vec = np.array([1.0, 0.0, 0.0]) is maximum forward
        #   vec = np.array([0.0, 0.0, 0.0]) is stop
        #   vec = np.array([1.0, 0.0, 1.0]) is a right (clockwise) turn at maximum speed

        # From MR documentation:
        # If the norm of the vector [<force_x>,<force_y>,<torque_z>] is greater than 1,
        # the vector should be normalized in order to ensure valid values.
        if np.linalg.norm(vec) > 1:
            vec = vec / np.linalg.norm(vec)
        vec = vec.tolist()  # convert numpy.array into a list

        msg = self.format_cmd(msg_id='MARMAN', payload=vec)
        self.stream.sendall(msg)

    # Send a course command
    def send_cmd_course(self, course, speed):
        msg = self.format_cmd(msg_id='MARCRS', payload=[course, speed])
        self.stream.sendall(msg)

    # Send a drift command (i.e., turn off all motors and just move with the current)
    def send_cmd_drift(self):
        msg = self.format_cmd(msg_id='MARABT')
        self.stream.sendall(msg)

    # Send a station mode command
    def send_cmd_station(self, msg):
        lat = msg.position.latitude
        lon = msg.position.longitude

        if lat < 0:
            lat_dir = 'S'
            lat = -1 * lat
        else:
            lat_dir = 'N'

        if lon < 0:
            lon_dir = 'W'
            lon = -1 * lon
        else:
            lon_dir = 'E'

        speed = msg.speed

        msg = self.format_cmd(msg_id='MARSTA',
                              payload=[lat, lat_dir, lon, lon_dir, speed])
        self.stream.sendall(msg)

    # Format command as NMEA message
    def format_cmd(self, talker='P', msg_id='MARABT', msg_mode=1, payload=[]):
        msg = nmea.NMEAMessage(talker=talker, msgID=msg_id,
                               msgmode=msg_mode, payload=payload)
        serial_msg = msg.serialize()
        return serial_msg


def main(args=None):
    rclpy.init(args=args)

    otter_sub = OtterSubscriber()

    rclpy.spin(otter_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    otter_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
