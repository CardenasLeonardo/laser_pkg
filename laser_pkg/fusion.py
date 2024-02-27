    #---------------------------------------------LIBRERIAS ------------------------------------------------------#
from math import pow, atan2, sqrt, asin, cos, sin
from math import radians, degrees
import math
import time
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist, Pose, Quaternion, Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class MoveRobotNode(Node):
    
    #---------------------------------------------- SETUP ---------------------------------------------------------#
    def __init__(self):

        #-----------------------SE INICIALIZA EL NODO Y SE IMPRIME UN MENSAJE -------------------------#
        super().__init__('move_robot_node')
        self.get_logger().info("Node move_robot_node Started")

        #----------------------------- GENERAR PUBLISHERS Y SUBSCRIBERS -------------------------------#
        
        self.cmd_vel_pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.laser_subs= self.create_subscription(LaserScan,'/scan',self.laserscan_callback,10)
        self.odom_subs= self.create_subscription(Odometry,'/odom',self.odom_callback,10)

        self.stop_robot()
        print('Coloca el robot en tu origen deseado')
        self.setup()
        

        #---------------------------------- SET VARIABLES EN CERO -------------------------------------#

        self.custom_origin = None
        self.initialized = False
        

        self.k1 = 0.5
        self.k2 = 1

        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.theta = 0.0
        self.goal_pose = Pose2D()
        self.goal_pose.x = 0.0
        self.goal_pose.y = 0.0
        
        self.start_path = False
        
        #self.odom_subs
        #ejecuta cada cierto tiempo
       


    #-------------------------------------------- END SETUP -------------------------------------------------------#

    def setup(self):
        respuesta = input("¿Estás listo para iniciar el programa? (s/n): ")
        if respuesta.lower() != 's':
            print("Programa no iniciado.")
            return
        print("Programa iniciado.")


    #----------------------------- FUNCION LLAMADA CADA QUE SE RECIBE MSG EN LASER --------------------------------#
    def laserscan_callback(self,msg):
        
        self.lidar_data = msg
        rango = msg.ranges
        umbral = 2.0
        mode = 1

        if mode == 0:
            self.mover(0.0 , 0.0)
            print('mode: ',mode)
        
        if mode == 1:
            #rango por derecha y frente 90 - 180
            a = 60
            b = 200
            #rango por derecha 90
            c = 70
            d = 110
            #Posicion de lado
            e = 90
            f = 1.0
            g = -1.0

        if mode == 2:
            #rango por derecha y frente 240 - 160
            a = 160
            b = 290
            #rango por izquierda 270
            c = 250
            d = 290
            #Posicion de lado
            e = 270
            f = 1.0
            g = 1.0
        


        minm = min(rango[a:b]) #Rango de monitoreo por derecha y frente
        min_index = rango.index(minm)  #Angulo de la menor medicion
        err = (min (rango[c:d])) - umbral #error d   Rango de monitoreo por la derecha
        erra = (min_index - e)/10 #  /10  es una constante
    
        if err>1:
            err=1.0

        if err<-1:
            err=-1.0

        print('error',err)
        print('error a',erra)
        print('mode: ',mode)
        self.mover(0.4 , erra*f + err*6*g)


        


        
    #-------------------------------- FUNCION LLAMADA CADA QUE SE RECIBE MSG EN ODOM -------------------------------#
    def odom_callback(self, msg_odom):
        position = msg_odom.pose.pose.position
        orientation = msg_odom.pose.pose.orientation
        (posx, posy, posz) = (position.x, position.y, position.z)
        #(qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
        #print('x:',posx,'y:',posy,'z',posz)
        if not self.initialized:
            self.custom_origin = (posx, posy, posz)
            self.initialized = True

        if self.custom_origin is not None:
            (x_origin, y_origin, z_origin) = self.custom_origin
            (posx_custom, posy_custom, posz_custom) = (posx - x_origin, posy - y_origin, posz - z_origin)
            #print('x:', posx_custom, 'y:', posy_custom, 'z:', posz_custom)



        
        
        


            
            
       

       

    
    #----------------------------------------- FUNCION PARA MOVER EL ROBOT -----------------------------------------#
    def mover(self,pot_l,pot_a): 
        msg = Twist()
        msg.linear.x  = pot_l
        msg.angular.z = pot_a
        self.cmd_vel_pub.publish(msg)
        print('vel linear: %f vel ang: %f' % (pot_l, pot_a))


    #-------------------------------------------- FUNCION PARA ERROR a ----------------------------------------------#
    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x),2)+pow((goal_pose.y-self.pose.y),2))
    


    #------------------------------------------- FUNCION PARA ERROR alpha -------------------------------------------#
    def alpha(self, goal_pose):

        alph = atan2(
            (goal_pose.y - self.pose.y),
            (goal_pose.x-self.pose.x)
        ) - 2*self.theta
        
        return alph
    

    #------------------------------------------- FUNCION PARA DETENER ROBOT ------------------------------------------#
    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        #publicar a velocidades
        self.cmd_vel_pub.publish(msg)
        print("ROBOT DETENIDO")
     




#------------ MAIN -----------#
def main(args=None):
    rclpy.init()
    move_rn = MoveRobotNode()               
    rclpy.spin(move_rn)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
#-----------------------------#
