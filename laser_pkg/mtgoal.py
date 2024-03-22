    #---------------------------------------------LIBRERIAS ------------------------------------------------------#
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



class MoveRobotNode(Node):
    
    #---------------------------------------------- SETUP ---------------------------------------------------------#
    def __init__(self):

        #-----------------------SE INICIALIZA EL NODO Y SE IMPRIME UN MENSAJE -------------------------#
        super().__init__('move_robot_node')
        self.get_logger().info("Node move_robot_node Started")
        
        #----------------------------- GENERAR PUBLISHERS Y SUBSCRIBERS -------------------------------#
        
        self.cmd_vel_pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.odom_subs= self.create_subscription(Odometry,'/odom',self.odom_callback,10)

        self.publisher_ = self.create_publisher(Int8, 'topic_orden', 10)
        self.subscription_colision = self.create_subscription(Int8,'topic_colision',self.colision_callback,10)

        

        #---------------------------------- SET VARIABLES EN CERO -------------------------------------#
      
        self.custom_origin = None
        self.initialized = False
        self.colision_detectada = False
        self.coordenada1 = None

        self.k1 = 0.3
        self.k2 = 0.2

        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.theta = 0.0
        self.goal_pose = Pose2D()
        self.goal_pose.x = 3.0
        self.goal_pose.y = 3.0
        self.colision_pose = Pose2D()
        
        self.start_path = False
        self.colision = False

        self.bandera = False
        #self.odom_subs
        #ejecuta cada cierto tiempo




    

    #-------------------------------- FUNCION LLAMADA CADA QUE SE RECIBE MSG EN ODOM -------------------------------#
    def odom_callback(self, msg_odom):

        if not self.initialized:
            #Set origen
            self.custom_origin = None
            self.select_origen(msg_odom)
            self.custom_origin = position.x , position.y
            self.initialized = True    

        if self.custom_origin is not None:
            
            #Coordenadas actuales
            x,y,orientation = self.coordenadas(msg_odom)
            
            #Coordenada incio de boundary follow 
            if self.colision and not self.bandera:
                self.colision_pose.x = x
                self.colision_pose.y = y

            #Calculo final de boundary follow
            

            v,w = self.ley_control(x,y,orientation)
            self.mover(v,w)

        """
        #--------- Inicio y termino de comportamiento boundary follow -----------#
        colision = self.colision
        colision_pose = self.colision_pose
        goal_pose = self.goal_pose
        

        #Guardar coordenada de colision
        if colision == True and self.estado == False:    
            self.colision_pose.x = posx_custom
            self.colision_pose.y = posy_custom
            self.estado = True

        #Calculo de fin de obstacle avoidance
        vectorA = np.array([(posx_custom - colision_pose.x),(posy_custom - colision_pose.y) ])
        vectorB = np.array([goal_pose.x - posx_custom , goal_pose.y - posy_custom] )
        vectorR = np.array([goal_pose.x - colision_pose.x, goal_pose.y - colision_pose.y])
        distanciaA = np.linalg.norm(vectorA)
        distanciaB = np.linalg.norm(vectorB)
        distanciaC = distanciaA+distanciaB
        distanciaR = np.linalg.norm(vectorR)

        print('goal pose: (',self.goal_pose.x ,' , ', self.goal_pose.y,')   robo pose: (',round(posx_custom,2),',', round(posy_custom,2),' ) ',' , ','colision_pose: (', colision_pose,')',theta,theta*180/math.pi,')')
        print('distancia referencia:',distanciaR, '    distanciaA:',distanciaA, '    distanciaB:',distanciaB,  '    distanciaC:',distanciaC, )
        
        
        
        


        if self.estado == False: 
            self.orden(0)
            self.mover(v,w)
        else :
            if distanciaC + 0.2 > distanciaR and distanciaC - 0.2 < distanciaR  :
                self.orden(0)
                self.mover(v,w)
            else :
                self.orden(1)
        
        """
    

    #----------------------------------- FUNCION PARA SETEAR ORIGEN  ------------------------------------#
    def select_origen(self,msg_odom):
     
        self.stop_robot()
        position = msg_odom.pose.pose.position
        orientation = msg_odom.pose.pose.orientation
        print('Coloca el robot en tu origen deseado')
        respuesta = input("¿Estás listo para iniciar el programa? (s/n): ")
        if respuesta.lower() != 's':
            print("Programa no iniciado.")

        
        if not self.initialized:
            self.custom_origin = (position.x , position.y)
            self.initialized = True

        if self.custom_origin is not None:
            (x_origin, y_origin) = self.custom_origin
            (posx_custom, posy_custom) = (position.x - x_origin, position.y - y_origin)
        return posx_custom, posy_custom,orientation
    
        

       
    #------------------------------ FUNCION PARA GENERAR COORDENADAS  ------------------------------------#
    def coordenadas(self,msg_odom):
        

    #-------------------------------- FUNCION PARA CALCULAR LEY DE CONTROL -------------------------------#
    def ley_control(self,x,y,orientation):
        
        #------------------------- tetha quaternion -----------------------------#
        (xq,yq,zq,wq) = (orientation.x, orientation.y, orientation.z, orientation.w)
        t3 = +2.0 * (wq*zq+xq*yq)
        t4 = +1.0 - 2.0 * (yq*yq+zq*zq)
        theta = math.atan2(t3,t4)

        #----------------------------- Error a ----------------------------------#
        xr = self.goal_pose.x
        yr = self.goal_pose.y
        xe = xr - x
        ye = yr - y
        a = sqrt(xe**2 + ye**2)

        #---------------------------- Error alpha -------------------------------#
        alpha = atan2(ye,xe) - theta 
        
        #-------------------------- Ley de control ------------------------------#
        v = self.k1 * a * cos(alpha)
        w = self.k2 * alpha + self.k1 * sin(alpha) * cos(alpha)
        print('goal pose: (',self.goal_pose.x ,' , ', self.goal_pose.y,')   robo pose: (',round(x,2),',', round(y,2),' ) ',' , ',theta,theta*180/math.pi,')')
        return v,w
    '''
    #--------------------------------------  FUNCION BANDERA RODEAR OBSTACULO ---------------------------------------#
    def bandera(self,msg_odom):

        position = msg_odom.pose.pose.position
        (posx, posy, posz) = (position.x, position.y, position.z)

         #--------- Inicio y termino de comportamiento boundary follow -----------#
        colision = self.colision
        colision_pose = self.colision_pose
        goal_pose = self.goal_pose
        

        #Guardar coordenada de colision
        if colision == True and self.estado == False:    
            self.colision_pose.x = posx_custom
            self.colision_pose.y = posy_custom
            self.estado = True

        #Calculo de fin de obstacle avoidance
        vectorA = np.array([(posx_custom - colision_pose.x),(posy_custom - colision_pose.y) ])
        vectorB = np.array([goal_pose.x - posx_custom , goal_pose.y - posy_custom] )
        vectorR = np.array([goal_pose.x - colision_pose.x, goal_pose.y - colision_pose.y])
        distanciaA = np.linalg.norm(vectorA)
        distanciaB = np.linalg.norm(vectorB)
        distanciaC = distanciaA+distanciaB
        distanciaR = np.linalg.norm(vectorR)

        print('goal pose: (',self.goal_pose.x ,' , ', self.goal_pose.y,')   robo pose: (',round(posx_custom,2),',', round(posy_custom,2),' ) ',' , ','colision_pose: (', colision_pose,')',theta,theta*180/math.pi,')')
        print('distancia referencia:',distanciaR, '    distanciaA:',distanciaA, '    distanciaB:',distanciaB,  '    distanciaC:',distanciaC, )
        
        '''

    #------------------------------------------ FUNCION PARA ENVIAR ORDEN -------------------------------------------#
    def orden(self,a):
        msg = Int8()
        msg.data = a
        self.publisher_.publish(msg)
        print('Publishing: ', msg.data)
        
    #------------------------------------------- Advertencia de colision ----------------------------------------#
    def colision_callback(self, msg):
        colision = msg.data
        if colision == 1:
            self.colision = True
        else : self.colision = False

    #----------------------------------------- FUNCION PARA MOVER EL ROBOT -----------------------------------------#
    def mover(self,pot_l,pot_a): 
        msg = Twist()
        msg.linear.x  = pot_l
        msg.angular.z = pot_a
        self.cmd_vel_pub.publish(msg)
    
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
#-----------------------------