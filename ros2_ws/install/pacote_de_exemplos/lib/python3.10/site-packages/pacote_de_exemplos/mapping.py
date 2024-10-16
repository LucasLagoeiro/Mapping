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

        self.get_logger().info ("Im on simulation")
        self.get_logger().debug ('Definindo o subscriber dda odometria: "/odom"')
        self.poseX, self.poseY, self.yaw = None, None, None
        self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)



    def listener_callback_laser(self, msg):
        self.laser = msg.ranges
        self.angle = msg.angle_increment
        self.angleMin = msg.angle_min
        self.angleMax = msg.angle_max


    def listener_callback_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        #------Position---------
        self.poseX, self.poseY = p.x, p.y
        #------Orientation---------
        self.yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


        
    def run(self):
         
        self.get_logger().debug ('Executando uma iteração do loop de processamento de mensagens.')
        rclpy.spin_once(self)

        self.get_logger().debug ('Definindo mensagens de controde do robô.')
        self.ir_para_frente = Twist(linear=Vector3(x= 0.05,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z= 0.0))
        self.parar          = Twist(linear=Vector3(x= 0.0,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z= 0.0))

        self.obstacles_x = np.array([])
        self.obstacles_y = np.array([])


        self.get_logger().info ('Entrando no loop princial do nó.')
        while(rclpy.ok):

            while (self.yaw == None): rclpy.spin_once(self)
            

            ang, dist = self.lidar_read()
            if len(ang) != len(dist): 
                continue
            else:
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
                plt.pause(0.05)  # Pausa para permitir a atualização do gráfico

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