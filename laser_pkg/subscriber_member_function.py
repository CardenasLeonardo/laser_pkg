import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from sensor_msgs.msg import LaserScan

class ReadingLaser(Node):

    def __init__(self):
        super().__init__('reading_laser')

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE



        self.subscription= self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile,
        )


    def listener_callback(self,msg):
        self.get_logger().info('angle_min: "%f" ' %(msg.angle_min))
        self.get_logger().info('angle_max: "%f" ' %(msg.angle_max))
        self.get_logger().info('angle_increment: "%f" ' %(msg.angle_increment))
        self.get_logger().info('time_increment: "%f" ' %(msg.time_increment))
        self.get_logger().info('scan_time: "%f" ' %(msg.scan_time))
        self.get_logger().info('range_min: "%f" ' %(msg.range_min))
        self.get_logger().info('range_max: "%f" ' %(msg.range_max))
        self.get_logger().info('numero ranges: "%f" ' %((len(msg.ranges))))
        if len(msg.ranges) > 0:
            # Utilizar un bucle para mostrar cada valor en la cadena de log
            log_string = 'ranges: '
            for value in msg.ranges:
                log_string += '"%f" ' % value
            # Imprimir la cadena de log
            self.get_logger().info(log_string)
        else:
            self.get_logger().info('El vector "ranges" está vacío.')
            

def main(args=None):
    rclpy.init()
    reading_laser = ReadingLaser()                  
    reading_laser.get_logger().info("Hello friend!")
    rclpy.spin(reading_laser)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reading_laser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()