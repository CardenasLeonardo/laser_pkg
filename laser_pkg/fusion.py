#--------------------------------------LIBRERIAS (ANEXAR LASER)-----------------------------------

from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time




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
    #--------------------------------------- END SETUP --------------------------------------------------




    #----------------------------- FUNCION LLAMADA CADA QUE SE RECIBE MSG EN LASER --------------------------------------------------
    def laserscan_callback(self,msg):
        # -------------- EL MSG de LaserScan se almacena en self.lidar_data ---------------------
        self.lidar_data = msg
        rango = msg.ranges
        umbral = 0.45
        minm = min(rango[60:200])
        min_index = rango.index(minm)

        err = (min (rango[70:110])) - umbral #error d
        erra = (min_index - 90.0)/10
     
        if err>1:
            err=1.0

        if err<-1:
            err=-1.0

        print(self.obstaculo(rango,umbral))
        print('error',err)
        print('error a',erra)
        


        self.mover(0.5 , erra - err*6)
      
            
            
       

       

    
    def mover(self,pot_l,pot_a): 

        msg = Twist()
        msg.linear.x  = pot_l
        msg.angular.z = pot_a
        self.cmd_vel_pub.publish(msg)
        print('vel linear: %f vel ang: %f' % (pot_l, pot_a))
     

    def obstaculo(self,rango,umbral):
        mensaje=''
        if any(r < umbral for r in rango[80:100]):
            mensaje = 'derecha' 
                 
        if any(r < umbral for r in rango[260:280]):
            mensaje = 'izquierda'
            
        if any(r < umbral for r in rango[170:190]):
            mensaje = 'centro'
                  
        return mensaje





def main(args=None):
    rclpy.init()
    move_rn = MoveRobotNode()               
    rclpy.spin(move_rn)
    rclpy.shutdown()



if __name__ == '__main__':
    main()