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
import matplotlib.pyplot as plt

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
        self.odom_subs= self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        
        self.k1 = 0.5
        self.k2 = 1

        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0

        self.theta = 0.0
        
        self.goal_pose = Pose2D()

        self.colision_pose = Pose2D()
        self.x_positions = []
        self.y_positions = []
        self.fig, self.ax = plt.subplots()

    # FUNCION LLAMADA CADA QUE SE RECIBE MSG EN ODOM
    def odom_callback(self, msg_odom):
        
        self.pose.x = round(msg_odom.pose.pose.position.x,4)
        self.pose.y = round(msg_odom.pose.pose.position.y,4)


        orientation = msg_odom.pose.pose.orientation
        
        (xq,yq,zq,wq) = (orientation.x, orientation.y, orientation.z, orientation.w)
        t3 = +2.0 * (wq*zq+xq*yq)
        t4 = +1.0 - 2.0 * (yq*yq+zq*zq)
        
        self.theta = math.atan2(t3,t4)
        self.plot_position()

    def plot_position(self):
        self.x_positions.append(self.pose.x)
        self.y_positions.append(self.pose.y)
        self.ax.clear()
        self.ax.grid(True)
        self.ax.plot(self.x_positions, self.y_positions)
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.set_title('Robot Position over Time')
        plt.pause(0.01)
    


#------------ MAIN -----------#
def main(args=None):
    rclpy.init()
    move_rn = MoveRobotNode()               
    rclpy.spin(move_rn)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
#-----------------------------#
