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

#--------------------

#El comentario general de que hace una clase va aqui
#Esta clase toma como valores de entrada ciertos parametros .. y regresa tal dato -- algo asi
class MoveRobotNode(Node):
    
    def __init__(self):
        
        # SE INICIALIZA EL NODO Y SE IMPRIME UN MENSAJE
        super().__init__('move_robot_node')
        self.get_logger().info("Node move_robot_node Started")
        
        # GENERAR PUBLISHERS Y SUBSCRIBERS
        self.cmd_vel_pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.odom_subs= self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.laser_subs= self.create_subscription(LaserScan,'/scan',self.laserscan_callback,10)
        
        #SET VARIABLES EN CERO
        
        self.custom_origin = None
        self.initialized = False
        self.colision_detectada = False
        self.coordenada1 = None

        self.k1 = 0.5
        self.k2 = 1

        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0

        self.theta = 0.0
        
        self.goal_pose = Pose2D()
        self.colision_pose = Pose2D()
        
        self.start_path = False
        self.colision = False

        self.bandera = False
        #self.odom_subs

        self.timer = self.create_timer(0.1, self.move2goal)

        self.obstacle = {'obstacle':False,'orientation':None}
        
    def laserscan_callback(self,msg):

        self.lidar_data = msg
        
        umbral = 2.0

        obstacle_right = any(r < umbral for r in msg.ranges[90:180])
        obstacle_left = any(r < umbral for r in msg.ranges[240:270])
        
        
        #crear un filtro para saber si es por der. o izq.
        if any(r < umbral for r in msg.ranges):

            self.obstacle['obstacle'] = True
            
            if obstacle_right:
                self.obstacle['orientation'] = 'D'
                self.get_logger().info('Derecha')
            if obstacle_left:
                self.obstacle['orientation'] = 'I'
                self.get_logger().info('Izquierda')
        else:

            self.obstacle['obstacle'] = False
            
            self.get_logger().info('TODO FINE')
        
        
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
                print(self.obstacle['orientation'])
                #v = 0.0
                #w = 0.0
            else:
                print('OK')
                
            v,w = self.ley_control(self.pose.x,self.pose.y,self.theta)
            
            self.mover(v,w)

        else:

            self.stop_robot()
            self.start_path = False

            answer = input("Quieres ingresar otro punto? y/n:   ")
            
            if answer == "y":
                pass
            else:
                #FIXME
                rclpy.shutdown()
                exit(-1)

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
            v = 0.8

        if v < -1:
            v = -0.8
        
        w = self.k2 * alpha + self.k1 * sin(alpha) * cos(alpha)
        
        if w > 1.0:
            w = 0.8

        if w < -1.0:
            w = -0.8
        
        print('goal pose: (',self.goal_pose.x ,' , ', self.goal_pose.y,')   robo pose: (',round(x,2),',', round(y,2),' ) ',' , ',theta,theta*180/math.pi,')')

        return v,w


    def follow_b(self):
        
        return v,w

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
        print("ROBOT DETENIDO")
        
#------------ MAIN -----------#
def main(args=None):

    print('Coloca el robot en tu origen deseado')
    respuesta = input("¿Estás listo para iniciar el programa? (s/n): ")

    if respuesta.lower() != 's':
        print("Programa no iniciado.")
    else:
        rclpy.init()
        move_rn = MoveRobotNode()
        rclpy.spin(move_rn)
        
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
#-----------------------------
