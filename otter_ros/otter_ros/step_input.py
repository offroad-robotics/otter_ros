import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Vector3, WrenchStamped


class StepInputPublisher(Node):

    def __init__(self):
        super().__init__('step_input_publisher')

        self.declare_parameter('input_level', 0.1)
        self.declare_parameter('delay', 10.0)

        self.i = 0
        # seconds of delay between activity
        self.delay = self.get_parameter('delay').value
        # input level of the step
        self.input_level = self.get_parameter('input_level').value

        self.timer_period = 0.1  # seconds

        self.publisher_ = self.create_publisher(
            WrenchStamped, 'control_cmds', 10)
        self.timer = self.create_timer(self.timer_period, self.step_sequence)

    def step_sequence(self):
        msg = WrenchStamped()
        if self.i < self.delay/self.timer_period:
            msg.wrench.force.x = 0.0
            msg.wrench.torque.z = 0.0
            self.i += 1
        elif self.i < 2 * (self.delay/self.timer_period):
            msg.wrench.force.x = self.input_level
            msg.wrench.torque.z = 0.0
            self.i += 1
        elif self.i < 3 * (self.delay/self.timer_period):
            msg.wrench.force.x = 0.0
            msg.wrench.torque.z = 0.0
            self.i += 1
        else:
            return
        self.publisher_.publish(msg)
        self.get_logger().info('Surge command: %s' % msg.wrench.force.x)


def main(args=None):
    rclpy.init(args=args)

    step_input_publisher = StepInputPublisher()

    rclpy.spin(step_input_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    step_input_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
