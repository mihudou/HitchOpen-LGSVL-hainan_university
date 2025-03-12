import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("publish_dummy_joy")
        self.publisher_ = self.create_publisher(Joy, "/joy", 1)
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Joy()
        msg.header.stamp = rclpy.clock.Clock().now().to_msg()

        control = [0.,0.,0,0,0,0,0,0]
        msg.buttons = control[2:]
        msg.axes = [-control[1],0.,control[0],1.,1.,-control[0]]
        
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
