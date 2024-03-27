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
import os,sys
import numpy as np
class MoveRobotNode(Node):
    
    
    def __init__(self):

        
        super().__init__('move_robot_node')
        # GENERAR PUBLISHERS Y SUBSCRIBERS 
        self.cmd_vel_pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.odom_subs= self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.laser_subs= self.create_subscription(LaserScan,'/scan',self.laserscan_callback,10)

        self.bandera = False
        self.rango = None
        self.timer = self.create_timer(0.05, self.prueba)
     
        self.v = 0.0
        self.w = 0.0
        self.mode = 0
        
        self.obstacle3 = False
        self.obstacle2 = False
        self.obstacle_right = False
        self.obstacle_left = False

        self.pose = Pose2D()  
        self.pose.x = 0.0
        self.pose.y = 0.0

        self.k1 = 0.5
        self.k2 = 1
        self.theta = 0.0
        self.goal_pose = Pose2D()

        self.colision_pose = Pose2D()
        self.colision_pose.x = 0.0
        self.colision_pose.y = 0.0
        
        self.salida_pose = Pose2D()
        self.salida_pose.x = 0.0
        self.salida_pose.y = 0.0

        self.start_path = False

        self.obstacle =  dict(obstacle=False,orientation=None)

        self.umbral = 1.0
        self.umbral2 = 1.6

        self.prev_ = 0

        self.first_time = True
        self.first_time_out = True
        
        self.colision = False
        self.flag_colision = False
        self.flag_salida = False
        self.rodeado = False
        self.redzone = False
        self.greenzone = False
    #FUNCION LLAMADA CADA QUE SE RECIBE MSG EN LASER 
    
    def laserscan_callback(self,msg):

        self.rango = msg.ranges
        self.obstacle2 = any(r < self.umbral for r in msg.ranges[90:270])
        self.obstacle3 = any(r < self.umbral2 for r in msg.ranges[90:270])
        self.obstacle_right = any(r < self.umbral for r in msg.ranges[90:179])
        self.obstacle_left = any(r < self.umbral for r in msg.ranges[181:270])
        
    # FUNCION LLAMADA CADA QUE SE RECIBE MSG EN ODOM
    def odom_callback(self, msg_odom):
        
        self.pose.x = round(msg_odom.pose.pose.position.x,4)
        self.pose.y = round(msg_odom.pose.pose.position.y,4)
        orientation = msg_odom.pose.pose.orientation
        
        (xq,yq,zq,wq) = (orientation.x, orientation.y, orientation.z, orientation.w)
        t3 = +2.0 * (wq*zq+xq*yq)
        t4 = +1.0 - 2.0 * (yq*yq+zq*zq)
        self.theta = math.atan2(t3,t4)
        
    # ERROR AL GOAL
    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x),2)+pow((goal_pose.y-self.pose.y),2))

    # BOUNDARY FOLLOW
    def boundary_follow(self):
        
        if not self.bandera:
            if self.obstacle_right:
                self.mode = 1
                self.bandera = True
            if self.obstacle_left:
                self.mode = 2
                self.bandera = True
        
        umbral = self.umbral
        rango = self.rango
        if self.mode == 1:
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

        if self.mode == 2:
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
        
        if self.mode>0:
            minm = min(rango[a:b])              #Medicion menor del Rango de monitoreo
            min_index = rango.index(minm)       #Angulo de la menor medicion
            err = (min (rango[c:d])) - umbral   #error a
            erra = (min_index - e)/10           #error alpha
        
            if err>1:
                err=1.0

            if err<-1:
                err=-1.0

            self.v = 0.4
            self.w = erra*f + err*6*g
            self.mover(self.v , self.w)



    # FUNCION PARA CALCULAR LEY DE CONTROL
    def ley_control(self,x,y,theta):

        # Error a
        x_err = self.goal_pose.x - x
        y_err = self.goal_pose.y - y
        a = sqrt(x_err**2 + y_err**2)

        if(theta > math.pi):
            theta = theta - 2 * math.pi
        if(theta < -math.pi):
            theta = theta + 2 * math.pi
            
        # Error alpha
        alpha = atan2(y_err,x_err) - theta 
        
        # Ley de control
        v = self.k1 * a * cos(alpha)

        if v > 1:
            v = 1.0
        if v < -1:
            v = -1.0
        
        w = self.k2 * alpha + self.k1 * sin(alpha) * cos(alpha)
        
        if w > 1.0:
            w = 1.0
        if w < -1.0:
            w = -1.0

        self.mover(v,w)        

    # MOVE TO GOAL
    def move2goal(self):

        #mover hacia el objetivo
        goal_pose = self.goal_pose
        
        if not self.start_path:

            #parar robot
            self.stop_robot()
            

            #Datos
            point_x,point_y = input("Ingrese las coordenadas x,y:: ").split(",")
            
            self.goal_pose.x = float(point_x)
            self.goal_pose.y = float(point_y)
            self.colision_pose.x = 0.0
            self.colision_pose.y = 0.0
            self.salida_pose.x = 0.0
            self.salida_pose.y = 0.0
            self.flag_colision = False
            self.flag_salida = False
            self.first_time = True
            self.first_time_out = True
            self.start_path = True
        
        distance_tolerance = 0.15
        
        if self.euclidean_distance(self.goal_pose) >= distance_tolerance:
            self.ley_control(self.pose.x,self.pose.y,self.theta)

        else:
            self.stop_robot()
            self.start_path = False

            while True:
                answer = input("Quieres ingresar otro punto? y/n:   ")
                
                if answer.lower() not in ('y', 'n'):
                    print("Error fancy")
                else:
                    break
                
            if answer == "y":
                pass
            else:
                sys.exit()

    # FUNCION CRITERIO PARA SALIR DE BOUNDARY FOLLOW
    def alineado(self):
        x1, y1 = self.colision_pose.x, self.colision_pose.y  # Coordenadas del punto A
        x2, y2 = self.pose.x, self.pose.y  # Coordenadas del punto B
        x3, y3  = self.goal_pose.x, self.goal_pose.y  # Coordenadas del punto C
        area = 0.5 * abs(x1*y2 + x2*y3 + x3*y1 - y1*x2 - y2*x3 - y3*x1)
        print(area)
        if area <= 0.1 and area >= - 0.1:
            self.rodeado = True
            print('alineado')
        else:   
            self.rodeado = False
            print('no alineado')

    # FUNCION PARA EVITAR CONFLICTOS ENTRE CAMBIO DE COMPORTAMIENTOS FINAL
    def transicion(self):
        self.stop_robot()
        rango = self.rango
        umbral = self.umbral 
        minm = min(rango)
        min_index = rango.index(minm)
        err = (min (rango)) - umbral #error d
        erra = (min_index -1)/100
     
        if err>1:
            err=1.0

        if err<-1:
            err=-1.0

        print('error',err)
        print('error a',erra)
        
        if erra <= 0.03: self.mover(0.4 , 0.0)
        else: self.mover(0.0 , erra )

    # FUNCION PARA GENERAR ZONA DE COLISION Y EVITAR CONFLICTOS EN CAMBIO DE COMPORTAMIENTO INICIAL
    def red_zone(self):
        if self.first_time and self.obstacle2:
            self.colision_pose.x = self.pose.x
            self.colision_pose.y = self.pose.y
            self.first_time = False
        #region de colision (distancia entre pose y colision)
        separacion =  math.sqrt((self.pose.x - self.colision_pose.x)**2 + (self.pose.y - self.colision_pose.y)**2 ) 
        if separacion < 0.5: self.redzone = True
        if separacion > 0.9: self.redzone = False

        print('colision:',self.colision_pose,'separacion:',separacion,'redzone:',self.redzone)
    
    # FUNCION PARA GENERAR ZONA DE SALIDA Y EVITAR CONFLICTOS EN CAMBIO DE COMPORTAMIENTO FINAL
    def green_zone(self):
        if self.first_time_out and self.rodeado and not self.redzone:
            self.salida_pose.x = self.pose.x
            self.salida_pose.y = self.pose.y
            self.first_time_out = False
        #region de salida (distancia entre pose y salida)
        separacion =  math.sqrt((self.pose.x - self.salida_pose.x)**2 + (self.pose.y - self.salida_pose.y)**2 ) 
        if separacion < 0.5: self.greenzone = True
        if separacion > 0.9: self.greenzone = False

        print('salida:',self.salida_pose,'separacion:',separacion,'redgreenzone:',self.greenzone)

    # FUNCION EN BUCLE
    def prueba (self):
        self.alineado() #Caclula si se encuentra alineado
        self.red_zone() 
        self.green_zone()  
            
        if not self.flag_colision and self.obstacle2: 
            self.flag_colision = True
        if self.flag_colision: 
            print('red flag')

        if not self.flag_salida and self.rodeado: 
            self.flag_salida = True
        if self.flag_salida: 
            print('green flag')
        
        if self.flag_colision:
            if self.redzone or not self.rodeado and not self.greenzone : self.boundary_follow()
            if not self.redzone:
                if self.rodeado and self.green_zone: 
                    self.move2goal()
                    if not self.obstacle3 : self.flag_colision = False
        else: self.move2goal()

            


    '''    
        if not self.flag_colision:
            self.move2goal()
            if self.obstacle2: self.flag_colision = True

        if self.flag_colision:
            self.boundary_follow()
            if self.rodeado: self.flag_colision = False
    '''
        
        
    

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