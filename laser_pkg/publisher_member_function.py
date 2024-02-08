import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile
import time 

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

    def movimiento(self):
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        vel = Twist()
        vel.linear.x = 0.0
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = 0.0
        self.publish_twist(vel)

        # Avance en x
        vel.linear.x = 0.5
        self.publish_twist(vel)
        time.sleep(2)

        # Detenerse
        vel.angular.z = 0.0
        vel.linear.x = 0.0
        self.publish_twist(vel)
        time.sleep(2)

        # Girar 90 grados a la derecha
        vel.angular.z = 1.0
        self.publish_twist(vel)
        time.sleep(2)

         # Detenerse
        vel.angular.z = 0.0
        vel.linear.x = 0.0
        self.publish_twist(vel)
        time.sleep(2)

        # Avance en x
        vel.linear.x = 0.5
        self.publish_twist(vel)
        time.sleep(2)

        # Detenerse
        vel.angular.z = 0.0
        vel.linear.x = 0.0
        self.publish_twist(vel)
        time.sleep(2)

        # Girar 90 grados a la derecha
        vel.angular.z = 1.0
        self.publish_twist(vel)
        time.sleep(2)

         # Detenerse
        vel.angular.z = 0.0
        vel.linear.x = 0.0
        self.publish_twist(vel)
        time.sleep(2)

        # Avance en x
        vel.linear.x = 0.5
        self.publish_twist(vel)
        time.sleep(2)

        # Detenerse
        vel.angular.z = 0.0
        vel.linear.x = 0.0
        self.publish_twist(vel)
        time.sleep(2)

        # Girar 90 grados a la derecha
        vel.angular.z = 1.0
        self.publish_twist(vel)
        time.sleep(2)

         # Detenerse
        vel.angular.z = 0.0
        vel.linear.x = 0.0
        self.publish_twist(vel)
        time.sleep(2)

        # Avance en x
        vel.linear.x = 0.5
        self.publish_twist(vel)
        time.sleep(2)

        # Detenerse
        vel.angular.z = 0.0
        vel.linear.x = 0.0
        self.publish_twist(vel)
        time.sleep(2)   


    def publish_twist(self, vel):
        self.publisher_.publish(vel)
        self.get_logger().info(f"Publicando Twist: {vel}")

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    minimal_publisher.movimiento()
    rclpy.spin(minimal_publisher)

    # Destruir el nodo explícitamente
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()