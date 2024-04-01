import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Vector3, WrenchStamped

import numpy as np


class SineInputPublisher(Node):

    def __init__(self):
        super().__init__('sine_input_publisher')

        self.declare_parameter('delay', 5.0)
        self.declare_parameter('input_frequency', 1.0)
        self.declare_parameter('input_amplitude', 0.5)

        self.i = 0
        # seconds of delay between activity
        self.delay = self.get_parameter('delay').value
        # input frequency of sinusoid
        self.input_frequency = self.get_parameter('input_frequency').value
        # amplitude of sinusoid
        self.input_amplitude = self.get_parameter('input_amplitude').value

        self.timer_period = 0.05  # seconds

        self.publisher_ = self.create_publisher(
            WrenchStamped, 'control_cmds', 10)
        self.timer = self.create_timer(self.timer_period, self.input_sequence)

    def input_sequence(self):
        msg = WrenchStamped()
        time = self.i * self.timer_period

        # First 'delay' seconds:
        if time < self.delay:
            msg.wrench.force.x = 0.0
            msg.wrench.torque.z = 0.0
            self.i += 1
        # Perform 10 cycles of sinusoid:
        elif time <= self.delay + 10 / self.input_frequency:
            msg.wrench.force.x = self.sine_function(time - self.delay)
            msg.wrench.torque.z = 0.0
            self.i += 1
        # Stop for delay period:
        elif time < 2 * self.delay + 10 / self.input_frequency:
            msg.wrench.force.x = 0.0
            msg.wrench.torque.z = 0.0
            self.i += 1
        # And then we are done, so stop publishing:
        else:
            return

        self.publisher_.publish(msg)
        self.get_logger().info('Surge command: %s' % msg.wrench.force.x)

    def sine_function(self, time):
        return self.input_amplitude * np.sin(2 * np.pi * self.input_frequency * time)


def main(args=None):
    rclpy.init(args=args)

    sine_input_publisher = SineInputPublisher()

    rclpy.spin(sine_input_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sine_input_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
