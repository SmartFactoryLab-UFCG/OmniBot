#!/usr/bin/env python3
import random
import time
from math import sin, cos, atan2, sqrt, fabs, pi

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry

# Import da biblioteca de rotação do Scipy
from scipy.spatial.transform import Rotation as R

# As funções de conversão manuais foram removidas


def normalize(z):
    return atan2(sin(z), cos(z))


def angle_diff(a, b):
    a = normalize(a)
    b = normalize(b)
    d1 = a - b
    d2 = 2 * pi - fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if fabs(d1) < fabs(d2):
        return d1
    else:
        return d2


class OdometryMotionModel(Node):
    def __init__(self):
        super().__init__('odometry_motion_model')
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.last_odom_theta = 0.0
        self.is_first_odom = True

        self.declare_parameter('alpha1', 0.01)
        self.declare_parameter('alpha2', 0.01)
        self.declare_parameter('alpha3', 0.01)
        self.declare_parameter('alpha4', 0.01)
        self.declare_parameter('nr_samples', 30)

        self.alpha1 = self.get_parameter('alpha1').get_parameter_value().double_value
        self.alpha2 = self.get_parameter('alpha2').get_parameter_value().double_value
        self.alpha3 = self.get_parameter('alpha3').get_parameter_value().double_value
        self.alpha4 = self.get_parameter('alpha4').get_parameter_value().double_value
        self.nr_samples = self.get_parameter('nr_samples').get_parameter_value().integer_value

        if self.nr_samples >= 0:
            self.samples = PoseArray()
            self.samples.poses = [Pose() for _ in range(self.nr_samples)]
        else:
            self.get_logger().fatal('Invalid Number of Samples requested: %d! Exit..', self.nr_samples)
            return

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.pose_array_pub = self.create_publisher(
            PoseArray, 'odom_motion_model/samples', 10
        )

    def odom_callback(self, odom):
        q = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
             odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
        
        # --- [ALTERAÇÃO] Conversão de Quaternião para Euler com Scipy ---
        rot = R.from_quat(q)
        _, _, yaw = rot.as_euler('xyz') # 'xyz' corresponde a roll, pitch, yaw

        if self.is_first_odom:
            self.samples.header.frame_id = odom.header.frame_id
            self.last_odom_x = odom.pose.pose.position.x
            self.last_odom_y = odom.pose.pose.position.y
            self.last_odom_theta = yaw
            self.is_first_odom = False
            return
            
        odom_x_increment = odom.pose.pose.position.x - self.last_odom_x
        odom_y_increment = odom.pose.pose.position.y - self.last_odom_y
        odom_theta_increment = angle_diff(yaw, self.last_odom_theta)

        if sqrt(pow(odom_y_increment, 2) + pow(odom_x_increment, 2)) < 0.01:
            delta_rot1 = 0.0
        else:
            delta_rot1 = angle_diff(atan2(odom_y_increment, odom_x_increment), self.last_odom_theta)
            
        delta_trans = sqrt(pow(odom_x_increment, 2) + pow(odom_y_increment, 2))
        delta_rot2 = angle_diff(odom_theta_increment, delta_rot1)

        rot1_variance = self.alpha1 * abs(delta_rot1) + self.alpha2 * delta_trans
        trans_variance = self.alpha3 * delta_trans + self.alpha4 * (abs(delta_rot1) + abs(delta_rot2))
        rot2_variance = self.alpha1 * abs(delta_rot2) + self.alpha2 * delta_trans
        
        random.seed(int(time.time()))

        for i in range(len(self.samples.poses)):
            rot1_noise = random.gauss(0.0, rot1_variance)
            trans_noise = random.gauss(0.0, trans_variance)
            rot2_noise = random.gauss(0.0, rot2_variance)
            
            delta_rot1_draw = angle_diff(delta_rot1, rot1_noise)
            delta_trans_draw = delta_trans - trans_noise
            delta_rot2_draw = angle_diff(delta_rot2, rot2_noise)

            sample_q = [self.samples.poses[i].orientation.x, self.samples.poses[i].orientation.y,
                        self.samples.poses[i].orientation.z, self.samples.poses[i].orientation.w]
            
            # --- [ALTERAÇÃO] Conversão de Quaternião para Euler com Scipy ---
            sample_rot = R.from_quat(sample_q)
            _, _, sample_yaw = sample_rot.as_euler('xyz')

            self.samples.poses[i].position.x += delta_trans_draw * cos(sample_yaw + delta_rot1_draw)
            self.samples.poses[i].position.y += delta_trans_draw * sin(sample_yaw + delta_rot1_draw)
            
            # --- [ALTERAÇÃO] Conversão de Euler para Quaternião com Scipy ---
            new_rot = R.from_euler('z', sample_yaw + delta_rot1_draw + delta_rot2_draw)
            q = new_rot.as_quat() # Retorna um array numpy [x, y, z, w]
            
            self.samples.poses[i].orientation.x = q[0]
            self.samples.poses[i].orientation.y = q[1]
            self.samples.poses[i].orientation.z = q[2]
            self.samples.poses[i].orientation.w = q[3]

        self.last_odom_x = odom.pose.pose.position.x
        self.last_odom_y = odom.pose.pose.position.y
        self.last_odom_theta = yaw

        self.pose_array_pub.publish(self.samples)


def main():
    rclpy.init()
    node = OdometryMotionModel()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()