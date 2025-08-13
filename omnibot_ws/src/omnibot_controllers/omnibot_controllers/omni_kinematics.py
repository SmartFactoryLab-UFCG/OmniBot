#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from rclpy.time import Time

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry, Path
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
import numpy as np
import math

class OmniKinematics(Node):
    """
    1. Cinemática Inversa: Converte comandos /cmd_vel (Twist) em velocidades para as rodas.
    2. Odometria: Converte os estados das juntas (/joint_states) em uma estimativa da pose do robô (Odometry) e publica a transformação TF.
    """
    def __init__(self):
        super().__init__("omni_kinematics")

        # --- Parâmetros ---
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("wheel_distance", 0.2)
        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_distance_ = self.get_parameter("wheel_distance").get_parameter_value().double_value
        r = self.wheel_radius_
        L = self.wheel_distance_
        self.get_logger().info(f"Using wheel radius (r): {r} m")
        self.get_logger().info(f"Using wheel distance (L): {L} m")
        
        # --- Variáveis de Odometria ---
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        self.wheel_prev_pos_ = np.zeros(3)
        self.prev_time_ = None
        self.joint_names_ = ["omni_wheel_joint_1", "omni_wheel_joint_2", "omni_wheel_joint_3"]

        # --- Publishers e Subscribers ---
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "omni_wheel_controller/commands", 10)
        self.odom_pub_ = self.create_publisher(Odometry, "odom", 10)
        self.vel_sub_ = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.joint_sub_ = self.create_subscription(JointState,"/joint_states", self.joint_callback, 10)
        self.path_pub_ = self.create_publisher(Path, "/path", 10)
        self.br_ = TransformBroadcaster(self)

        # --- Matrizes Cinemáticas ---
        # Matriz de cinemática inversa (Twist -> Velocidade das Rodas)
        self.inverse_kinematics_matrix_ = np.array([
            [-np.sqrt(3)/2,   1/2,  L],
            [0,             -1,    L],
            [np.sqrt(3)/2,   1/2,  L]
        ])
        # Matriz de cinemática direta (Velocidade das Rodas -> Twist)
        self.forward_kinematics_matrix_ = np.array([
            [-r/np.sqrt(3),   0,      r/np.sqrt(3)],
            [r/3,           -2*r/3,   r/3],
            [r/(3*L),       r/(3*L),  r/(3*L)]
        ])

        # --- NOVA MENSAGEM DE CAMINHO ---
        self.path_msg_ = Path()
        self.path_msg_.header.frame_id = "odom"

        self.get_logger().info("Omni Kinematics Node has been started.")

    def cmd_vel_callback(self, msg: Twist):
        # --- CINEMÁTICA INVERSA ---
        robot_vel_vector = np.array([[msg.linear.x], [msg.linear.y], [msg.angular.z]])
        wheel_speeds_rad_per_sec = (1 / self.wheel_radius_) * (self.inverse_kinematics_matrix_ @ robot_vel_vector)
        wheel_speeds_msg = Float64MultiArray()
        wheel_speeds_msg.data = wheel_speeds_rad_per_sec.flatten().tolist()
        self.wheel_cmd_pub_.publish(wheel_speeds_msg)

    def joint_callback(self, msg: JointState):
        # --- ODOMETRIA (CINEMÁTICA DIRETA) ---
        
        # Lida com a primeira mensagem recebida
        if self.prev_time_ is None:
            self.prev_time_ = Time.from_msg(msg.header.stamp)
            # Mapeia as posições iniciais para a nossa ordem correta
            for i, name in enumerate(self.joint_names_):
                if name in msg.name:
                    idx = msg.name.index(name)
                    self.wheel_prev_pos_[i] = msg.position[idx]
            return

        # 1. Calcula o delta de tempo e de posição das rodas
        current_time = Time.from_msg(msg.header.stamp)
        dt = (current_time - self.prev_time_).nanoseconds / S_TO_NS
        
        # Garante a ordem correta das juntas, independentemente da ordem em /joint_states
        current_pos = np.zeros(3)
        for i, name in enumerate(self.joint_names_):
            if name in msg.name:
                idx = msg.name.index(name)
                current_pos[i] = msg.position[idx]
        
        dp = current_pos - self.wheel_prev_pos_

        # 2. Calcula a velocidade angular de cada roda (rad/s)
        if dt > 0:
            wheel_velocities = dp / dt
        else:
            return

        # 3. Calcula a velocidade do robô (vx, vy, v_theta) no seu próprio referencial
        robot_vel = self.forward_kinematics_matrix_ @ wheel_velocities
        vx_robot = robot_vel[0]
        vy_robot = robot_vel[1]
        v_theta_robot = robot_vel[2]

        # 4. Integra a pose usando o método de Euler
        # Primeiro, rotaciona a velocidade do referencial do robô para o referencial do mundo (odom)
        dx_world = (vx_robot * math.cos(self.theta_) - vy_robot * math.sin(self.theta_)) * dt
        dy_world = (vx_robot * math.sin(self.theta_) + vy_robot * math.cos(self.theta_)) * dt
        d_theta = v_theta_robot * dt

        self.x_ += dx_world
        self.y_ += dy_world
        self.theta_ += d_theta

        # 5. Prepara e publica a mensagem de Odometria
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        
        odom_msg.pose.pose.position.x = self.x_
        odom_msg.pose.pose.position.y = self.y_
        q = quaternion_from_euler(0, 0, self.theta_)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x = vx_robot
        odom_msg.twist.twist.linear.y = vy_robot
        odom_msg.twist.twist.angular.z = v_theta_robot
        self.odom_pub_.publish(odom_msg)

        # 6. Prepara e publica a transformação TF (odom -> base_footprint)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x_
        t.transform.translation.y = self.y_
        t.transform.rotation = odom_msg.pose.pose.orientation
        self.br_.sendTransform(t)

        # Atualiza as variáveis para a próxima iteração
        self.prev_time_ = current_time
        self.wheel_prev_pos_ = current_pos

        # --- ATUALIZAR E PUBLICAR O CAMINHO ---
        # 7. Cria uma nova pose com timestamp
        current_pose = PoseStamped()
        current_pose.header.stamp = current_time.to_msg()
        current_pose.header.frame_id = "odom"
        current_pose.pose.position.x = self.x_
        current_pose.pose.position.y = self.y_
        current_pose.pose.orientation = odom_msg.pose.pose.orientation

        # 8. Adiciona a nova pose à lista de poses na mensagem Path
        self.path_msg_.header.stamp = current_time.to_msg()
        self.path_msg_.poses.append(current_pose)
        
        # 9. Publica a mensagem de caminho completa
        self.path_pub_.publish(self.path_msg_)

        # Atualiza as variáveis para a próxima iteração
        self.prev_time_ = current_time
        self.wheel_prev_pos_ = current_pos


def main(args=None):
    rclpy.init(args=args)
    omni_kinematics_node = OmniKinematics()
    rclpy.spin(omni_kinematics_node)
    omni_kinematics_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()