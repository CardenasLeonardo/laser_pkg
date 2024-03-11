from ast import In
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int8


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(Float32,'topic_mtgoal',self.listener_callback,10)
        self.publisher_ = self.create_publisher(Int8, 'topic_mode_m2g', 10)


    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%f"' % msg.data)
        if msg.data < 0.05 :
            self.m2g_mode(0)
        else : self.m2g_mode(1)

    

    def m2g_mode(self,a):
        msg = Int8()
        msg.data = a
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%i"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()