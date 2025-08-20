#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import math

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

def wrap_to_pi(angle):
    """Transforma um ângulo em radianos para o intervalo [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi

class PointStabilizationController(Node):
    def __init__(self):
        super().__init__('point_stabilization_controller')

        # --- Parâmetros ---
        self.declare_parameter('use_ground_truth', False)
        self.declare_parameter('goal_pose.x', 0.0)
        self.declare_parameter('goal_pose.y', 0.0)
        self.declare_parameter('goal_pose.theta', 0.0)
        self.declare_parameter('gains.k1', 0.5)
        self.declare_parameter('gains.k2', 0.5)
        self.declare_parameter('gains.kth', 0.5)

        use_ground_truth = self.get_parameter('use_ground_truth').get_parameter_value().bool_value
        
        # --- Parâmetros do Controlador ---
        self.xd_ = self.get_parameter('goal_pose.x').get_parameter_value().double_value
        self.yd_ = self.get_parameter('goal_pose.y').get_parameter_value().double_value
        self.thetad_ = self.get_parameter('goal_pose.theta').get_parameter_value().double_value
        self.k1_ = self.get_parameter('gains.k1').get_parameter_value().double_value
        self.k2_ = self.get_parameter('gains.k2').get_parameter_value().double_value
        self.kth_ = self.get_parameter('gains.kth').get_parameter_value().double_value
        
        self.get_logger().info(f"Goal Pose: (x={self.xd_}, y={self.yd_}, theta={self.thetad_})")
        self.get_logger().info(f"Gains: k1={self.k1_}, k2={self.k2_}, kth={self.kth_}")

        # --- Publishers e Subscribers ---
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Escolhe qual tópico de pose subscrever com base no parâmetro
        if use_ground_truth:
            pose_topic = '/ground_truth_pose' # Tópico da ground-truth
            self.get_logger().info("A usar a Ground Truth da simulação.")
        else:
            pose_topic = '/odom' # Tópico da odometria calculada
            self.get_logger().info("A usar a odometria do robô.")
        
        self.pose_sub_ = self.create_subscription(
            Odometry,
            pose_topic,
            self.pose_callback,
            10
        )

        self.tf_static_broadcaster_ = StaticTransformBroadcaster(self)
        # Cria a mensagem de transformação
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom' # O pai do nosso goal é o frame odom
        t.child_frame_id = 'goal_frame' # O nome do nosso novo frame
        
        # Define a translação (posição) do goal
        t.transform.translation.x = self.xd_
        t.transform.translation.y = self.yd_
        t.transform.translation.z = 0.0 # Assumindo um robô 2D
        
        # Define a rotação do goal
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.thetad_ /2.0)
        t.transform.rotation.w = math.cos(self.thetad_ /2.0)
        
        # Envia a transformação (só precisa de ser feito uma vez)
        self.tf_static_broadcaster_.sendTransform(t)
        self.get_logger().info(f"A publicar o 'goal_frame' estático em relação ao 'odom'")



    def pose_callback(self, msg: Odometry):
        # Extrai a pose atual da mensagem de odometria
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # Converte o quaternion para o ângulo de Euler (theta/yaw)
        current_theta = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))

        # --- Implementação do Algoritmo de Controle (baseado no PDF) ---

        # 1. Calcular o erro de orientação [cite: 79]
        e_theta = wrap_to_pi(current_theta - self.thetad_)

        # 2. Calcular o erro de posição no referencial do mundo
        error_world = np.array([current_x - self.xd_, current_y - self.yd_])

        # 3. Rotacionar o erro para o referencial do robô [cite: 78]
        R_T = np.array([
            [math.cos(current_theta), math.sin(current_theta)],
            [-math.sin(current_theta), math.cos(current_theta)]
        ])
        e_body = R_T @ error_world
        ex = e_body[0]
        ey = e_body[1]

        # 4. Calcular as velocidades de controle [cite: 87, 88]
        vx = -self.k1_ * ex
        vy = -self.k2_ * ey
        omega = -self.kth_ * e_theta

        # 5. Criar e publicar a mensagem de comando de velocidade
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = vx
        cmd_vel_msg.linear.y = vy
        cmd_vel_msg.angular.z = omega
        self.cmd_vel_pub_.publish(cmd_vel_msg)


def main(args=None):
    rclpy.init(args=args)
    controller_node = PointStabilizationController()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()