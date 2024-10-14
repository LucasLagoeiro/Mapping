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


class R2D2(Node):

    def __init__(self):
        super().__init__('R2D2')
        self.get_logger().debug ('Definido o nome do nó para "R2D2"')

        qos_profile = QoSProfile(depth=10, reliability = QoSReliabilityPolicy.BEST_EFFORT)

        self.get_logger().debug ('Definindo o subscriber do laser: "/scan"')
        self.laser = None
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)


        # self.get_logger().debug ('Definindo o subscriber do laser: "/odom"')
        # self.pose = None
        # self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)

        self.get_logger().debug ('Definindo o publisher de controle do robo: "/cmd_Vel"')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # self.get_logger().info ('Definindo buffer, listener e on_timertimer para acessar as TFs.')
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)

        # self.declare_parameter('inSimulation', True)
        # inSimulation = self.get_parameter('inSimulation').get_parameter_value().bool_value
        
        # if inSimulation: 
        #     self.get_logger().info ("Im on simulation")
        #     self.timer = self.create_timer(0.1, self.on_timer)

        # else:
        #     self.get_logger().info ("Im on real robot")
        #     self.get_logger().debug ('Definindo o subscriber do laser: "/joint_states"')
        #     self.right_yaw = None
        #     self.left_yaw = None
        #     self.create_subscription(JointState, '/joint_states', self.listener_callback_jointState, qos_profile)


    def listener_callback_laser(self, msg):
        # if min(msg.ranges[50:70]) is not None or min(msg.ranges[ 0:10]) is not None or min(msg.ranges[170:190]) is not None:
        #     self.laser = msg.ranges
        #     self.get_logger().info ('LaserLeft: ' + str(min(self.laser[50:70])) + " LaserFront: " + str(min(self.laser[0:10])) + "LaserRight: " + str(min(self.laser[170:190])))
        self.laser = msg.ranges
        self.angle = msg.angle_increment
        self.angleMin = msg.angle_min - pi/2
        self.angleMax = msg.angle_max


    
    # def listener_callback_jointState(self, msg):
    #     # # The state of each joint (revolute or prismatic) is defined by:
	# 	#  * the position of the joint (rad or m),
	# 	#  * the velocity of the joint (rad/s or m/s) and 
	# 	#  * the effort that is applied in the joint (Nm or N).
        
    #     self.get_logger().info("Valores dos encoders: " + str(msg.position[0]) + str(msg.position[1]))


    #     self.left_yaw = msg.position[0]
    #     self.right_yaw = msg.position[1]
        
		
       
    # def listener_callback_odom(self, msg):
    #     self.pose = msg.pose.pose

		


    # def on_timer(self):
    #     try:
    #         self.tf_right = self.tf_buffer.lookup_transform(
    #             "right_center_wheel",
    #             "right_leg_base",
    #             rclpy.time.Time())

    #         _, _, self.right_yaw = tf_transformations.euler_from_quaternion(
    #             [self.tf_right.transform.rotation.x, self.tf_right.transform.rotation.y, 
    #             self.tf_right.transform.rotation.z, self.tf_right.transform.rotation.w]) 
            

    #         self.get_logger().info (
    #             f'yaw right_leg_base to right_center_wheel: {self.right_yaw}')
            
            

    #     except TransformException as ex:
    #         self.get_logger().info(
    #         f'Could not transform right_leg_base to right_center_wheel: {ex}')
        
        
    #     try:
    #         self.tf_left = self.tf_buffer.lookup_transform(
    #             "left_center_wheel",
    #             "left_leg_base",
    #             rclpy.time.Time())

    #         _, _, self.left_yaw = tf_transformations.euler_from_quaternion(
    #             [self.tf_left.transform.rotation.x, self.tf_left.transform.rotation.y, 
    #             self.tf_left.transform.rotation.z, self.tf_left.transform.rotation.w]) 
            

    #         self.get_logger().info (
    #             f'yaw left_leg_base to left_center_wheel: {self.left_yaw}')
            

    #     except TransformException as ex:
    #         self.get_logger().info(
    #         f'Could not transform left_leg_base to left_center_wheel: {ex}')

        
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



        self.get_logger().info ('Entrando no loop princial do nó.')
        laserCount = 0
        while(rclpy.ok):
            self.angles = self.angleMin + self.angle * laserCount
            ang, dist = self.angles, self.laser[laserCount]
            ox = np.sin(ang) * dist
            oy = np.cos(ang) * dist
            plt.clf()  # Limpa o gráfico anterior para evitar sobreposição
            plt.plot([oy, np.zeros(np.size(oy))], [ox, np.zeros(np.size(oy))], "ro-") # lines from 0,0 to the
            plt.axis("equal")
            bottom, top = plt.ylim()  # return the current ylim
            plt.ylim((top, bottom)) # rescale y axis, to match the grid orientation
            plt.grid(True)
            # plt.show()
            plt.pause(0.000001)  # Pausa para permitir a atualização do gráfico
            # break
            laserCount+= 1
            if laserCount == len(self.laser): laserCount = 0
            

        

        self.get_logger().info ('Ordenando o robô: "parar"')
        # self.pub_cmd_vel.publish(self.parar)
        rclpy.spin_once(self)


    def lidar_read(self):
        anglesList = []
        distances = []
        laserCount = 0
        for laserCount in range(len(self.laser)):
            rclpy.spin_once(self)
            self.angles = self.angleMin + self.angle * laserCount
            anglesList.append(float(self.angles))
            distances.append(float(self.laser[laserCount]))
            laserCount+= 1
            
        angles = np.array(anglesList)
        distances = np.array(distances)
        return angles, distances

        

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

