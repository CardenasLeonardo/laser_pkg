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
from std_msgs.msg import Float32, Int8


class MoveRobotNode(Node):
    
    #---------------------------------------------- SETUP ---------------------------------------------------------#
    def __init__(self):

        #-----------------------SE INICIALIZA EL NODO Y SE IMPRIME UN MENSAJE -------------------------#
        super().__init__('move_robot_node')
        #----------------------------- GENERAR PUBLISHERS Y SUBSCRIBERS -------------------------------#
        self.cmd_vel_pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.laser_subs= self.create_subscription(LaserScan,'/scan',self.laserscan_callback,10)

        self.publisher_ = self.create_publisher(Int8, 'topic_colision', 10)
        self.subscription_colision = self.create_subscription(Int8,'topic_orden',self.order_callback,10)

    #----------------------------- FUNCION LLAMADA CADA QUE SE RECIBE MSG EN LASER --------------------------------#
    
    def laserscan_callback(self,msg):
        
        self.lidar_data = msg
        rango = msg.ranges
        umbral = 2.0
        mode = 1

        if any(r < umbral for r in rango):
            self.colision(1)
        else: 
            self.colision(0)    


        if mode == 0:
            self.mover(0.0 , 0.0)
            
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
            #rango por izquierda y frente 240 - 160
            a = 160
            b = 290
            #rango por izquierda 270
            c = 250
            d = 290
            #Posicion de lado
            e = 270
            f = 1.0
            g = 1.0
        
        minm = min(rango[a:b])              #Medicion menor del Rango de monitoreo
        min_index = rango.index(minm)       #Angulo de la menor medicion
        err = (min (rango[c:d])) - umbral   #error a
        erra = (min_index - e)/10           #error alpha
    
        if err>1:
            err=1.0

        if err<-1:
            err=-1.0

        #print(' a:', round(err,2), '    /   alpha:', erra, '    /   mode:', mode, '     /    orden:')
        #self.mover(0.4 , erra*f + err*6*g)
        global v,w
        v = 0.4
        w = erra*f + err*6*g
    

     


    #------------------------------------------------- Modo ordenado -----------------------------------------------#
    def order_callback(self, msg):
        orden = msg.data
        print('Order: ', orden)
        if orden == 1:
            self.mover(v,w)
        

    #---------------------------- ----------- FUNCION PARA PUBLICAR COLISION ---------------------------------------#
    def colision (self,a):
        msg = Int8()
        msg.data = a
        self.publisher_.publish(msg)
        print('sending colision alarm: ',a)
    #----------------------------------------- FUNCION PARA MOVER EL ROBOT -----------------------------------------#
    def mover (self,pot_l,pot_a): 
        msg = Twist()
        msg.linear.x  = pot_l
        msg.angular.z = pot_a
        self.cmd_vel_pub.publish(msg)
    #------------------------------------------ FUNCION PARA DETENER ROBOT -----------------------------------------#
    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        #publicar a velocidades
        self.cmd_vel_pub.publish(msg)
     




#------------ MAIN -----------#
def main(args=None):
    rclpy.init()
    move_rn = MoveRobotNode()               
    rclpy.spin(move_rn)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
#-----------------------------#