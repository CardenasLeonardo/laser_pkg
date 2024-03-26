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

class MoveRobotNode(Node):
    
    
    def __init__(self):

        
        super().__init__('move_robot_node')
        # GENERAR PUBLISHERS Y SUBSCRIBERS 
        self.cmd_vel_pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.odom_subs= self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.laser_subs= self.create_subscription(LaserScan,'/scan',self.laserscan_callback,10)

        self.bandera = False
        self.rango = None
        self.timer = self.create_timer(0.1, self.prueba)
        self.v = 0.0
        self.w = 0.0
        self.mode = 0
        self.umbral = 2.0
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
        
        self.start_path = False

        self.obstacle =  dict(obstacle=False,orientation=None)

        self.umbral = 1.5
        
        #self.timer = self.create_timer(0.1, self.boundary_follow)

        self.prev_ = 0

        self.first_time = True

    #FUNCION LLAMADA CADA QUE SE RECIBE MSG EN LASER 
    
    def laserscan_callback(self,msg):

        self.rango = msg.ranges
        self.obstacle2 = any(r < self.umbral for r in msg.ranges[90:270])
        self.obstacle_right = any(r < self.umbral for r in msg.ranges[90:179])
        self.obstacle_left = any(r < self.umbral for r in msg.ranges[181:270])
        

    def boundary_follow(self):
        
        if not self.bandera:
            if self.obstacle_right:
                self.mode = 2
                self.bandera = True
            if self.obstacle_left:
                self.mode = 1
                self.bandera = True
        
        umbral = 0.6
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


    # FUNCION LLAMADA CADA QUE SE RECIBE MSG EN ODOM
    def odom_callback(self, msg_odom):
        
        self.pose.x = round(msg_odom.pose.pose.position.x,4)
        self.pose.y = round(msg_odom.pose.pose.position.y,4)
        orientation = msg_odom.pose.pose.orientation
        
        (xq,yq,zq,wq) = (orientation.x, orientation.y, orientation.z, orientation.w)
        t3 = +2.0 * (wq*zq+xq*yq)
        t4 = +1.0 - 2.0 * (yq*yq+zq*zq)
        self.theta = math.atan2(t3,t4)
        
    # error hacia el objetivo
    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x),2)+pow((goal_pose.y-self.pose.y),2))

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

        #publica cada 1seg
        self.get_logger().info(f'goal pose: ({self.goal_pose.x} , {self.goal_pose.y}) - robo pose: ({round(x,2)},{round(y,2)}) - theta: {round(theta,2)} - {round(theta*180/math.pi,2)}', throttle_duration_sec=0.5)

        #print('goal pose: (',self.goal_pose.x ,' , ', self.goal_pose.y,')   robo pose: (',round(x,2),',', round(y,2),' ) ',' , ',theta,theta*180/math.pi,')')

        self.mover(v,w)        

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


    def prueba (self):
        self.move2goal()
        
        """       
        if self.obstacle:   
            self.boundary_follow()
            self.mover(self.v , self.w)
        else:
            self.mover(0.8 , 0.0)
        """


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