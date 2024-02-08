import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time 

class ReadingLaser(Node):

    def __init__(self):
        super().__init__('reading_laser')

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.subscription= self.create_subscription(LaserScan,'/scan',self.listener_callback,qos_profile,
        )

        

    def movimiento(self):
        
        vel = Twist()
        
        vel.linear.x = 1.0
        self.publish_twist(vel)
    
    def publish_twist(self, vel):
        self.publisher_.publish(vel)
        self.get_logger().info(f"Publicando Twist: {vel}")

    def listener_callback(self,msg):
        self.lidar_data = msg
        self.get_logger().info('angle_min: "%f" ' %(self.lidar_data.angle_min))
        self.get_logger().info('angle_max: "%f" ' %(self.lidar_data.angle_max))
        self.get_logger().info('angle_increment: "%f" ' %(self.lidar_data.angle_increment))
        self.get_logger().info('time_increment: "%f" ' %(self.lidar_data.time_increment))
        self.get_logger().info('scan_time: "%f" ' %(self.lidar_data.scan_time))
        self.get_logger().info('range_min: "%f" ' %(self.lidar_data.range_min))
        self.get_logger().info('range_max: "%f" ' %(self.lidar_data.range_max))
        self.get_logger().info('numero ranges: "%f" ' %((len(self.lidar_data.ranges))))
        if len(self.lidar_data.ranges) > 0:
            # Utilizar un bucle para mostrar cada valor en la cadena de log
            log_string = 'ranges: '
            for value in self.lidar_data.ranges:
                log_string += '"%f" ' % value
            # Imprimir la cadena de log
            self.get_logger().info(log_string)
        else:
            self.get_logger().info('El vector "ranges" está vacío.')

    
            

def main(args=None):
    rclpy.init()
    reading_laser = ReadingLaser()                  
    rclpy.spin(reading_laser)
    

    # Destroy the node explicitly

    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reading_laser.destroy_node()
    reading_laser.movimiento()
    rclpy.spin(reading_laser)
    rclpy.shutdown()

if __name__ == '__main__':
    main()