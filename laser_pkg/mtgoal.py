#---------------------------------------------LIBRERIAS ------------------------------------------------------#
# No pongas notas con "---" tan largos
from math import pow, atan2, sqrt, asin, cos, sin, atan
from math import radians, degrees
import math
import time
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist, Pose, Quaternion, Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int8
import numpy as np

import os,sys

"""
Para bloques largos usa comillas 
#El comentario general de que hace una clase va aqui
#Esta clase toma como valores de entrada ciertos parametros .. y regresa tal dato -- algo asi
"""
class MoveRobotNode(Node):
    
    def __init__(self):
        
        # SE INICIALIZA EL NODO Y SE IMPRIME UN MENSAJE
        super().__init__('move_robot_node')
        
        self.get_logger().info("Node move_robot_node Started")
        
        # GENERAR PUBLISHERS Y SUBSCRIBERS
        self.cmd_vel_pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.odom_subs= self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.laser_subs= self.create_subscription(LaserScan,'/scan',self.laserscan_callback,10)
        
        self.k1 = 0.5
        self.k2 = 1

        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0

        self.theta = 0.0
        
        self.goal_pose = Pose2D()

        self.colision_pose = Pose2D()
        
        self.start_path = False

        self.obstacle =  dict(obstacle=False,orientation=None)

        self.umbral = 1.0
        
        self.timer = self.create_timer(0.1, self.move2goal)

        self.prev_ = 0

        self.first_time = True
        
    def laserscan_callback(self,msg):

        #self.obstacle['obstacle'] = False
        
        obstacle_right = any(r < self.umbral for r in msg.ranges[90:179])
        obstacle_left = any(r < self.umbral for r in msg.ranges[181:270])

        self.obstacle['min_right'] = min(msg.ranges[90:179])
        self.obstacle['max_right'] = max(msg.ranges[90:179])
        
        self.obstacle['min_left'] = min(msg.ranges[181:270])
        self.obstacle['max_left'] = max(msg.ranges[181:270])
        
        if obstacle_right:
            self.obstacle['obstacle'] = True
            self.obstacle['orientation'] = 'derecha'
                                    
        if obstacle_left:
            
            self.obstacle['obstacle'] = True
            self.obstacle['orientation'] = 'izquierda'
                        
    # FUNCION LLAMADA CADA QUE SE RECIBE MSG EN ODOM
    def odom_callback(self, msg_odom):
        
        self.pose.x = round(msg_odom.pose.pose.position.x,4)
        self.pose.y = round(msg_odom.pose.pose.position.y,4)

        orientation = msg_odom.pose.pose.orientation
        
        # tetha quaternion
        (xq,yq,zq,wq) = (orientation.x, orientation.y, orientation.z, orientation.w)
        t3 = +2.0 * (wq*zq+xq*yq)
        t4 = +1.0 - 2.0 * (yq*yq+zq*zq)
        
        self.theta = math.atan2(t3,t4)
        
    # error hacia el objetivo
    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x),2)+pow((goal_pose.y-self.pose.y),2))
    
    def move2goal(self):

        #mover hacia el objetivo
        goal_pose = self.goal_pose
        
        if not self.start_path:
            
            #Datos
            point_x,point_y = input("Ingrese las coordenadas x,y:: ").split(",")
            
            self.goal_pose.x = float(point_x)
            self.goal_pose.y = float(point_y)

            self.start_path = True
        
        distance_tolerance = 0.15
        
        if self.euclidean_distance(self.goal_pose) >= distance_tolerance:
            
            if self.obstacle['obstacle'] == True:
                
                self.follow_b()
                
            else:

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
                

    def follow_b(self):
        

        if self.first_time:
            self.colision_pose.x = self.pose.x
            self.colision_pose.y = self.pose.y

            self.first_time = False

        # vel angular
        # izq negativa
        # der positiva
            
        v = 0.5
        
        lado = self.obstacle['orientation']

        if lado == 'derecha':
            error = -1 * ((self.obstacle['min_right'] -  self.umbral) * 2)
        else:
            error = (self.obstacle['min_left'] -  self.umbral) * 2
                                
        if error>1:
            error=1.0
            
        if error<-1:
            error=-1.0

        w = error

        self.get_logger().info(f'error - vel angular: {w}', throttle_duration_sec=1)
        
        self.mover(v,w)

        #--------------------------------------------------------------------
        # Calculo cambio de signo en calculos con vectores
        #--------------------------------------------------------------------

        n = []
        
     	#Calcular Vector XoXf
        vx = self.goal_pose.x - self.colision_pose.x
        vy = self.goal_pose.y - self.colision_pose.y
        
     	#rotar
        n.append(-vy)
        n.append(vx)
        
        #ODOM - X0
        ODO_X0 = self.pose.x - self.colision_pose.x
        ODO_Y0 = self.pose.y - self.colision_pose.y
        
        #MULTIPLICAR
        calculos_signo = (ODO_X0*n[0]) + (ODO_Y0*n[1])
        
        #print(str(calculos_signo))
        #no deberia cambiar
        #print(str(self.colision_pose.x)+','+str(self.colision_pose.y))
	
        if((calculos_signo >= 0 and self.prev_ < 0) or (calculos_signo < 0 and self.prev_ >= 0)):
            self.obstacle["obstacle"] = False
            #self.first_time = True
            self.get_logger().info("CAMBIO SIGNO")
        
        self.prev_ = calculos_signo
        
        #---------------------------------------------------------------------

        
    # FUNCION PARA CALCULAR LEY DE CONTROL
    def ley_control(self,x,y,theta):

        # Error a
        x_err = self.goal_pose.x - x
        y_err = self.goal_pose.y - y
    
        a = sqrt(x_err**2 + y_err**2)

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

        #publica cada 1seg
        self.get_logger().info(f'goal pose: ({self.goal_pose.x} , {self.goal_pose.y}) - robo pose: ({round(x,2)},{round(y,2)}) - theta: {round(theta,2)} - {round(theta*180/math.pi,2)}', throttle_duration_sec=1)

        #print('goal pose: (',self.goal_pose.x ,' , ', self.goal_pose.y,')   robo pose: (',round(x,2),',', round(y,2),' ) ',' , ',theta,theta*180/math.pi,')')

        self.mover(v,w)

    # FUNCION PARA MOVER EL ROBOT
    def mover(self,pot_l,pot_a): 
        msg = Twist()
        msg.linear.x  = pot_l
        msg.linear.y = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = pot_a
        self.cmd_vel_pub.publish(msg)
    
    # FUNCION PARA DETENER ROBOT
    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info("ROBOT DETENIDO")
        
#------------ MAIN -----------#
def main(args=None):

    #limpiar pantalla
    os.system('clear')

    while True:
        
        print('Coloca el robot en tu origen deseado')
        respuesta = input("¿Estás listo para iniciar el programa? (s/n): ")

        if respuesta.lower() not in ('s', 'n'):
            print("Error fancy")
        else:
            break
        
        
    if respuesta.lower() != 's':
        print("Programa no iniciado.")
    else:

        rclpy.init()

        move_rn = MoveRobotNode()

        #FIXME - Salir del programa ctrl + c
        try:
            rclpy.spin(move_rn)
        except:
            sys.exit()
            
            
if __name__ == '__main__':
    main()
#-----------------------------
