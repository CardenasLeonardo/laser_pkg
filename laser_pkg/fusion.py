#--------------------------------------LIBRERIAS (ANEXAR LASER)-----------------------------------

from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan




class MoveRobotNode(Node):
    
    #--------------------------------------- SETUP ---------------------------------------------------------#
    def __init__(self):
        #--------------------------SE INICIALIZA EL NODO Y SE IMPRIME UN MENSAJE -------------------------------#
        super().__init__('move_robot_node')
        self.get_logger().info("Node move_robot_node Started")

        #------------------------------- GENERAR PUBLISHERS Y SUBSCRIBERS --------------------------------------#
        #Se crea un publisher que publica en el topic /cmd_vel mensajes del tipo Twist
        self.cmd_vel_pub = self.create_publisher(Twist,'/cmd_vel',10)

        #Se crea un subscriber en el topic /laser de mensajes tipo LaserScan
        #El argumento self.laserscan_callback es la funcion que se llamara cada que se reciba un msg
        self.laser_subs= self.create_subscription(LaserScan,'/scan',self.laserscan_callback,10)

        #--------------------------------- Set velocidades en 0 ------------------------------------------------#
        

    #--------------------------------------- END SETUP --------------------------------------------------




    #----------------------------- FUNCION LLAMADA CADA QUE SE RECIBE MSG EN LASER --------------------------------------------------
    def laserscan_callback(self,msg):
        # -------------- EL MSG de LaserScan se almacena en self.lidar_data ---------------------
        self.lidar_data = msg
        #------------- Se imprimen los valores de sel.lidar_data (contiene el msg)------
        self.imprimir_valores()
        ranges = msg.ranges

        umbral = 1.0
        
        obstacle_right = any(r < umbral for r in ranges[60:120])
        obstacle_center = any(r < umbral for r in ranges[160:200])
        obstacle_left = any(r < umbral for r in ranges[240:310])

        if obstacle_center:
            self.stop_robot()
            self.turnr()
           

        else :
            self.avance_robot()   
            

    #-------------------------- FUNCION LLAMADA PARA MOSTRAR INFO DEL LIDAR -----------------------------------------
    def imprimir_valores(self):
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

    
    def move(self):

        msg = Twist()
        msg.linear.x  = self.linear_velocity
        msg.angular.z = self.angular_velocity
        self.publisher.publish(msg)
        self.get_logger().info('Moviendo el robot:: veloc linear: %f velc ang: %f' % (self.linear_velocity, self.angular_velocity))
     

    def stop_robot(self):

        print("STOPING ROBOT")
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        #publicar a velocidades
        self.cmd_vel_pub.publish(msg)
        print("ROBOT DETENIDO")

    def avance_robot(self):

        print("STARTING ROBOT")
        msg = Twist()
        msg.linear.x = 0.6
        msg.angular.z = 0.0
        #publicar a velocidades
        self.cmd_vel_pub.publish(msg)
        print("ROBOT AVANZANDO")
    
    def turnr(self):

        print("ROTATING R ROBOT")
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.6
        #publicar a velocidades
        self.cmd_vel_pub.publish(msg)
        print("ROBOT GIRADO R")

    def turnl(self):

        print("ROTATING L ROBOT")
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = -0.6
        #publicar a velocidades
        self.cmd_vel_pub.publish(msg)
        print("ROBOT GIRADO L")    
    




def main(args=None):
    rclpy.init()
    move_rn = MoveRobotNode()               
    rclpy.spin(move_rn)
    rclpy.shutdown()



if __name__ == '__main__':
    main()
