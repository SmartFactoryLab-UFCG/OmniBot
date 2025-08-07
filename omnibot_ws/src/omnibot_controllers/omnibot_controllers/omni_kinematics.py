#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import numpy as np

class OmniKinematics(Node):
    """
    Este nó calcula a cinemática inversa para um robô omnidirecional de 3 rodas.
    Ele subscreve um tópico cmd_vel (Twist) e publica as velocidades
    individuais das rodas (Float64MultiArray) para um controlador ros2_control.
    """
    def __init__(self):
        super().__init__("omni_kinematics")

        # Declara os parâmetros do robô com valores padrão
        self.declare_parameter("wheel_radius", 0.03)  # Raio da roda (r) em metros
        self.declare_parameter("wheel_distance", 0.085) # Distância do centro à roda (L) em metros

        # Obtém os parâmetros
        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_distance_ = self.get_parameter("wheel_distance").get_parameter_value().double_value
        r = self.wheel_radius_
        L = self.wheel_distance_

        # Imprime os parâmetros utilizados
        self.get_logger().info(f"Using wheel radius (r): {r} m")
        self.get_logger().info(f"Using wheel distance (L): {L} m")

        # Cria o publisher para o controlador de velocidade das rodas
        self.wheel_cmd_pub_ = self.create_publisher(
            Float64MultiArray, 
            "omni_wheel_controller/commands", 
            10)

        # Cria o subscriber para os comandos de velocidade do robô
        self.vel_sub_ = self.create_subscription(
            Twist, 
            "omnibot_controller/cmd_vel", 
            self.cmd_vel_callback, 
            10)
        
        self.get_logger().info("Omni Kinematics Node has been started.")
        self.get_logger().info("Subscribing to /cmd_vel")
        self.get_logger().info("Publishing to omni_wheel_controller/commands")

        # Pré-calcula a matriz de cinemática inversa M 
        self.inverse_kinematics_matrix_ = np.array([
            [-np.sqrt(3)/2,   1/2,  L],
            [0,             -1,    L],
            [np.sqrt(3)/2,   1/2,  L]
        ])

    def cmd_vel_callback(self, msg: Twist):
        # 1. Cria o vetor de velocidade do robô a partir da mensagem Twist
        # [x_dot, y_dot, theta_dot]
        robot_vel_vector = np.array([[msg.linear.x],
                                     [msg.linear.y],
                                     [msg.angular.z]])

        # 2. Calcula as velocidades das rodas (w1, w2, w3)
        # [w1,w2,w3]: (1/r) * M @ v_robo
        wheel_speeds = (1 / self.wheel_radius_) * (self.inverse_kinematics_matrix_ @ robot_vel_vector)

        # 3. Prepara a mensagem para publicação
        wheel_speeds_msg = Float64MultiArray()
        # Converte o resultado do numpy array para uma lista python
        wheel_speeds_msg.data = wheel_speeds.flatten().tolist()
        
        # 4. Publica as velocidades calculadas para o controlador das rodas
        self.wheel_cmd_pub_.publish(wheel_speeds_msg)


def main():
    rclpy.init()

    omni_kinematics_node = OmniKinematics()
    rclpy.spin(omni_kinematics_node)

    omni_kinematics_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()