import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3


from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations
import numpy as np
from numpy import random
from math import *
import matplotlib.pyplot as plt
import time 

import math
from math import cos, sin, radians, pi
from pacote_de_exemplos import lidar_to_grid_map as lg
from pynput import keyboard


class R2D2(Node):

    def __init__(self):
        super().__init__('R2D2')
        self.get_logger().debug ('Definido o nome do nó para "R2D2"')

        qos_profile = QoSProfile(depth=10, reliability = QoSReliabilityPolicy.BEST_EFFORT)

        self.get_logger().debug ('Definindo o subscriber do laser: "/scan"')
        self.laser = None
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)

        self.get_logger().debug ('Definindo o publisher de controle do robo: "/cmd_Vel"')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.declare_parameter('inSimulation', True)
        inSimulation = self.get_parameter('inSimulation').get_parameter_value().bool_value
        
        if inSimulation: 
            self.get_logger().info ("Im on simulation")
            self.get_logger().debug ('Definindo o subscriber dda odometria: "/odom"')
            self.poseX, self.poseY, self.yaw = None, None, None
            self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)

            self.left_yaw, self.right_yaw = None, None

        else:
            self.get_logger().info ("Im on real robot")

            self.get_logger().debug ('Definindo o subscriber do laser: "/joint_states"')
            self.left_yaw, self.right_yaw = None, None
            self.create_subscription(JointState, '/joint_states', self.listener_callback_jointState, qos_profile)

            self.update()
            
            self.yaw = None
            self.subscription = self.create_subscription(Imu, '/imu', self.imu_callback, 10)




    def listener_callback_laser(self, msg):
        self.laser = msg.ranges
        self.angle = msg.angle_increment
        self.angleMin = msg.angle_min
        self.angleMax = msg.angle_max

    def listener_callback_jointState(self, msg):
        # # The state of each joint (revolute or prismatic) is defined by:
		#  * the position of the joint (rad or m),
		#  * the velocity of the joint (rad/s or m/s) and 
		#  * the effort that is applied in the joint (Nm or N).
        
        self.get_logger().info("Valores dos encoders: " + str(msg.position[0]) + str(msg.position[1]))


        self.left_yaw = msg.position[0]
        self.right_yaw = msg.position[1]
        


    def listener_callback_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        #------Position---------
        self.poseX, self.poseY = p.x, p.y
        #------Orientation---------
        self.yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def imu_callback(self, msg: Imu):
        # Estimate Yaw in Euler angles
        self.yaw = tf_transformations.euler_from_quaternion(msg.orientation)[2]


        
    def run(self):
         
        self.get_logger().debug ('Executando uma iteração do loop de processamento de mensagens.')
        rclpy.spin_once(self)

        self.get_logger().debug ('Definindo mensagens de controde do robô.')
        self.ir_para_frente = Twist(linear=Vector3(x= 0.05,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z= 0.0))
        self.parar          = Twist(linear=Vector3(x= 0.0,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z= 0.0))

        # self.get_logger().info ('Ordenando o robô: "ir para a frente"')
        # self.pub_cmd_vel.publish(self.ir_para_frente)
        # rclpy.spin_once(self)
        
        self.left_encoder = self.left_yaw
        self.right_encoder = self.right_yaw

        self.obstacles_x = np.array([])
        self.obstacles_y = np.array([])


        self.get_logger().info ('Entrando no loop princial do nó.')
        while(rclpy.ok):

            while (self.yaw == None): rclpy.spin_once(self)
            
            # self.get_logger().info ('Ang min: ' + str(self.angleMin) + " Yaw: " + str(self.yaw) + " Ref: " + str(self.angleMin+self.yaw))
            # self.get_logger().info ('X: "' + str(self.poseX) + "Y: " + str(self.poseY) + "Yaw:" + str(self.yaw))

            # rclpy.spin_once(self)

            ang, dist = self.lidar_read()
            self.ox_local = np.cos(ang) * dist 
            self.oy_local = np.sin(ang) * dist

            # Transformando coordenadas do laser para coordenadas do robô
            x_global = self.ox_local * np.cos(self.yaw) - self.oy_local * np.sin(self.yaw) + self.poseX
            y_global = self.ox_local * np.sin(self.yaw) + self.oy_local * np.cos(self.yaw) + self.poseY

            self.obstacles_x = np.append(self.obstacles_x, x_global)
            self.obstacles_y = np.append(self.obstacles_y,y_global)

            # ------Plot-------
            plt.clf()  # Limpa o gráfico anterior para evitar sobreposição
            plt.plot([self.obstacles_x], [self.obstacles_y], "ro-", label="Obstacle") # Objetos detectados pelo laser
            plt.plot(self.poseX,self.poseY, "ob", label="Robot")# Plota a posição do robô atual
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.005)  # Pausa para permitir a atualização do gráfico


            


        

        self.get_logger().info ('Ordenando o robô: "parar"')
        # self.pub_cmd_vel.publish(self.parar)
        rclpy.spin_once(self)


    def lidar_read(self):
        anglesList = []
        laserCount = 0
        for laserCount in range(len(self.laser)):
            rclpy.spin_once(self)
            self.angles = self.angleMin + (self.angle * laserCount)
            anglesList.append(float(self.angles))
            
        angles = np.array(anglesList)
        return angles, np.array(self.laser)

    def update(self):
        self.medidas[0] = self.left_encoder
        self.medidas[1] = self.right_encoder
        self.diff = self.medidas[0] - self.ultimas_medidas[0] # conta quanto a roda LEFT girou desde a última medida (rad)
        self.distancias[0] = self.diff * self.raio + random.normal(0,0.002) # determina distância percorrida em metros e adiciona um pequeno erro
        self.ultimas_medidas[0] = self.medidas[0]
        self.diff = self.medidas[1] - self.ultimas_medidas[1] # conta quanto a roda LEFT girou desde a última medida (rad)
        self.distancias[1] = self.diff * self.raio + random.normal(0,0.002) # determina distância percorrida em metros + pequeno erro
        self.ultimas_medidas[1] = self.medidas[1]
        # ## cálculo da dist linear e angular percorrida no timestep
        self.deltaS = (self.distancias[0] + self.distancias[1]) / 2.0
        self.deltaTheta = (self.distancias[1] - self.distancias[0]) / self.distancia_rodas
        self.pose_robot[2] = (self.pose_robot[2] + self.deltaTheta) % 6.28 # atualiza o valor Theta (diferença da divisão por 2π)
        # decomposição x e y baseado no ângulo
        self.deltaSx = self.deltaS * cos(self.pose_robot[2])
        self.deltaSy = self.deltaS * sin(self.pose_robot[2])
        # atualização acumulativa da posição x e y
        self.pose_robot[0] = self.pose_robot[0] + self.deltaSx # atualiza x
        self.pose_robot[1] = self.pose_robot[1] + self.deltaSy # atualiza y
        self.poseX = self.pose_robot[0]
        self.poseY = self.pose_robot[0]
        print("Postura:", self.pose_robot)



    # Destrutor do nó
    def __del__(self):
        self.get_logger().info('Finalizando o nó! Tchau, tchau...')


# Função principal
def main(args=None):
    # time.sleep(3)
    rclpy.init(args=args)
    node = R2D2()
    try:
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
   
if __name__ == '__main__':
    main()  

