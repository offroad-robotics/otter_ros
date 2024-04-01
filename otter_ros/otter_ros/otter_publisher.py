"""OtterROS Node - Otter Publisher (otter_publisher)

Publish telemetry from the Otter USV "OBS" to ROS topics.

**Requires the customized "pynmeagps" package included in the Otter ROS repository**

"""

# Import the necessary packages
# ROS2 packages
import rclpy
from rclpy.node import Node

# ROS2 interfaces
from sensor_msgs.msg import Imu, NavSatFix
from otter_msgs.msg import OtterStatus, COGSOG, OtterTime

# Third party python packages
import socket
import math

# Custom version of pynmeagps package
from pynmeagps import NMEAReader


class OtterPublisher(Node):

    def __init__(self):
        super().__init__('otter_publisher')

        # Adjustable node settings
        self.declare_parameter('port', 2009)  # flag for parametization
        # flag for parametization
        self.declare_parameter('obs_ip_address', "192.168.53.2")
        self.declare_parameter('frame_id', 'otter')

        # Topics to publish
        self.gps_publisher = self.create_publisher(
            NavSatFix, 'otter_gps', 10)

        self.gps_time_publisher = self.create_publisher(
            OtterTime, 'otter_gps_time', 10)

        self.imu_publisher = self.create_publisher(
            Imu, 'otter_imu', 10)

        self.status_publisher = self.create_publisher(
            OtterStatus, 'otter_status', 10)

        self.cogsog_publisher = self.create_publisher(
            COGSOG, 'otter_cogsog', 10)

        # Connect to the UDP stream
        self.stream = self.socket_connect()
        self.reader = NMEAReader(self.stream)

        self.get_logger().info('otter_publisher node initialized.')
        self.read_msgs()

    def socket_connect(self):
        """Connect to Otter socket and return the stream."""
        stream = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        stream.connect((self.get_parameter('obs_ip_address').value,
                       self.get_parameter('port').value))
        return stream

    def read_msgs(self):
        """ Read Otter data from UDP stream and publish to ROS topics."""
        for (_, parsed) in self.reader.iterate():
            time = self.get_clock().now().to_msg()

            # Option 1: NMEA GPS message
            if parsed.msgID == 'MARGPS':
                # $PMARGPS,<time>,<lat>,<lat_dir>,<lon>,<lon_dir>,<alt>,<sog>,<cog>*<checksum><CR><LF>

                # Publish GPS time message
                gps_time_msg = OtterTime()
                gps_time_msg.header.stamp = time

                gps_time = parsed.time
                gps_time_msg.time = float(gps_time)

                self.gps_time_publisher.publish(gps_time_msg)

                # Publish GPS message
                gps = NavSatFix()
                gps.header.stamp = time

                gps.header.frame_id = 'gps_frame'
                if parsed.lat_dir == 'S':
                    gps.latitude = -1*parsed.lat
                else:
                    gps.latitude = parsed.lat
                if parsed.lon_dir == 'W':
                    gps.longitude = -1*parsed.lon
                else:
                    gps.longitude = parsed.lon
                gps.altitude = parsed.alt

                self.gps_publisher.publish(gps)

                # Publish COGSOG message
                cogsog = COGSOG()
                cogsog.header.stamp = time

                if parsed.cog:
                    cogsog.cog = float(parsed.cog)
                    cogsog.sog = float(parsed.sog)

                    twist_x = float(parsed.sog) * \
                        math.cos(math.radians(float(parsed.cog)))
                    twist_y = float(parsed.sog) * \
                        math.sin(math.radians(float(parsed.cog)))
                    cogsog.twist.linear.x = twist_x
                    cogsog.twist.linear.y = twist_y

                self.cogsog_publisher.publish(cogsog)

            # Option 2: NMEA IMU message
            elif parsed.msgID == 'MARIMU':
                # $PMARIMU,<roll>,<pitch>,<yaw>,<p>,<q>,<r>*<checksum><CR><LF>

                # Publish IMU message
                imu = Imu()
                imu.header.stamp = time

                imu.header.frame_id = 'imu_frame'
                imu.orientation.x = parsed.roll
                imu.orientation.y = parsed.pitch
                imu.orientation.z = parsed.yaw
                if parsed.p != '':
                    # flag for review; why float here but not above?
                    imu.angular_velocity.x = float(parsed.p)
                    imu.angular_velocity.y = float(parsed.q)
                    imu.angular_velocity.z = float(parsed.r)

                self.imu_publisher.publish(imu)

            # Option 3: NMEA Otter status message
            elif parsed.msgID == 'MAROTR':
                # $PMAROTR,<rpm_port>,<rpm_stb>,<temp_port>,<temp_stb>,<power_port>,<power_stb>,<power_total>,<batteries_count>*<checksum><CR><LF>

                msg = OtterStatus()
                msg.header.stamp = time

                if parsed.temp_port:
                    msg.port.temp = parsed.temp_port
                    msg.starboard.temp = parsed.temp_stb
                msg.port.rpm = parsed.rpm_port
                msg.port.power = parsed.power_port
                msg.starboard.rpm = parsed.rpm_stb
                msg.starboard.power = parsed.power_stb
                msg.power = parsed.power_total
                msg.batteries = int(parsed.batteries_count)

                self.status_publisher.publish(msg)

            # Option 4: NMEA Otter mode/battery message
            elif parsed.msgID == 'MARMOD':
                # $PMARMOD,<mode>,<fuel_capacity>*<checksum><CR><LF>

                # To be implemented
                pass

            # Option 5: NMEA Otter error message
            elif parsed.msgID == 'MARERR':
                # $PMARERR,<description>*<checksum><CR><LF>

                # To be implemented
                pass


def main(args=None):
    rclpy.init(args=args)

    otter = OtterPublisher()

    rclpy.spin(otter)

    otter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
