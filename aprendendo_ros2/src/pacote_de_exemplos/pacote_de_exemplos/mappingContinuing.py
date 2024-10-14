import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, JointState
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


class R2D2(Node):

    def __init__(self):
        super().__init__('R2D2')
        self.get_logger().debug ('Definido o nome do nó para "R2D2"')
        plt.axis("equal")
        plt.gca().set_aspect("equal", "box")
        bottom, top = plt.ylim()  # return the current y-lim
        plt.ylim((top, bottom))  # rescale y axis, to match the grid orientation
        plt.grid(True)

        # plt.figure(figsize=(20,8))


        qos_profile = QoSProfile(depth=10, reliability = QoSReliabilityPolicy.BEST_EFFORT)

        self.get_logger().debug ('Definindo o subscriber do laser: "/scan"')
        self.laser = None
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)

        self.get_logger().debug ('Definindo o publisher de controle do robo: "/cmd_Vel"')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.declare_parameter('inSimulation', True)
        inSimulation = self.get_parameter('inSimulation').get_parameter_value().bool_value
        
        if inSimulation: 
            self.get_logger().info ("Im in simulation")
            self.get_logger().debug ('Definindo o subscriber do laser: "/odom"')
            self.poseX, self.poseY, self.yaw = None, None, None
            self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)

        else:
            self.get_logger().info ("Im on real robot")
            self.get_logger().debug ('Definindo o subscriber do laser: "/joint_states"')
            self.right_yaw = None
            self.left_yaw = None
            self.create_subscription(JointState, '/joint_states', self.listener_callback_jointState, qos_profile)


    def listener_callback_laser(self, msg):
        # if min(msg.ranges[50:70]) is not None or min(msg.ranges[ 0:10]) is not None or min(msg.ranges[170:190]) is not None:
        #     self.laser = msg.ranges
        #     self.get_logger().info ('LaserLeft: ' + str(min(self.laser[50:70])) + " LaserFront: " + str(min(self.laser[0:10])) + "LaserRight: " + str(min(self.laser[170:190])))
        self.laser = msg.ranges
        self.angle = -msg.angle_increment
        self.angleMin = -msg.angle_min
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


        		

        
    def run(self):
         
        self.get_logger().debug ('Executando uma iteração do loop de processamento de mensagens.')
        rclpy.spin_once(self)

        self.get_logger().debug ('Definindo mensagens de controde do robô.')
        self.ir_para_frente = Twist(linear=Vector3(x= 0.05,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z= 0.0))
        self.parar          = Twist(linear=Vector3(x= 0.0,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z= 0.0))

        # self.get_logger().info ('Ordenando o robô: "ir para a frente"')
        # self.pub_cmd_vel.publish(self.ir_para_frente)
        # rclpy.spin_once(self)

        # self.left_encoder = self.left_yaw
        # self.right_encoder = self.right_yaw


        xyreso_real = 0.02  # x-y grid resolution
        self.get_logger().info ('Entrando no loop princial do nó.')
        while(rclpy.ok):
            while (self.yaw == None): rclpy.spin_once(self)
            # self.get_logger().info ('X: "' + str(self.poseX) + "Y: " + str(self.poseY) + "Yaw:" + str(self.yaw))
            
            yawreso = math.radians(3.1)  # yaw angle resolution [rad]
            ang, dist = self.lidar_read()
            ox = np.sin(ang) * dist + self.poseX
            oy = np.cos(ang) * dist + self.poseY
            # plt.clf()  # Limpa o gráfico anterior para evitar sobreposição

            # pmap, minx, maxx, miny, maxy, xyreso = lg.generate_ray_casting_grid_map(ox, oy, xyreso_real, False)
            # xyres = np.array(pmap).shape
            
            # # plt.figure(figsize=(20,8))

            # # ------Map--------
            # plt.subplot(122) 
            # plt.imshow(pmap, cmap = "PiYG_r")
            # plt.clim(-0.4, 1.4)
            # plt.gca().set_xticks(np.arange(-.5, xyres[1], 1), minor = True)
            # plt.gca().set_yticks(np.arange(-.5, xyres[0], 1), minor = True)
            # plt.grid(True, which="minor", color="w", linewidth = .6, alpha = 0.5)
            # plt.colorbar()

            #------Lasers--------
            # plt.subplot(121)
            plt.plot([oy + self.poseY, np.zeros(np.size(oy))], [ox + self.poseX, np.zeros(np.size(oy))], "ro-")
            # plt.plot([oy], [ox], "ro-")
            plt.pause(0.001)
            
            # plt.plot([ox, np.zeros(np.size(oy))],[oy, np.zeros(np.size(oy))], "ro-")
            # plt.axis("equal")
            # plt.clf()  # Limpa o gráfico anterior para evitar sobreposição
            plt.plot(self.poseX, self.poseY, "ob")
            # plt.pause(0.5)  # Pausa para permitir a atualização do gráfico
            # plt.gca().set_aspect("equal", "box")
            # bottom, top = plt.ylim()  # return the current y-lim
            # plt.ylim((top, bottom))  # rescale y axis, to match the grid orientation
            # plt.grid(True)

            # plt.show()
            # # break
            
            

        

        self.get_logger().info ('Ordenando o robô: "parar"')
        # self.pub_cmd_vel.publish(self.parar)
        rclpy.spin_once(self)


    def lidar_read(self):
        anglesList = []
        distances = []
        laserCount = 0
        rclpy.spin_once(self)
        for laserCount in range(len(self.laser)):
            self.angles = (self.angle * laserCount) + (self.angleMin + self.yaw)
            anglesList.append(float(self.angles))
            distances.append(float(self.laser[laserCount]))
            laserCount+= 1
            
        angles = np.array(anglesList)
        distances = np.array(distances)
        return angles, distances
    
    # update function
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

