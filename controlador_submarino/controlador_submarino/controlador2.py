#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

# =========================
# PID
# =========================
class PID:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd

    def compute(self, error, derror):
        return self.kp * error - self.kd * derror

# =========================
# CONTROLADOR
# =========================
class Orca4Controller(Node):
    def __init__(self):
        super().__init__('orca4_controller')

        # Estado
        self.q = np.array([1, 0, 0, 0])
        self.omega = np.zeros(3)
        self.vel = np.zeros(3)

        # Deseado (ejemplo: yaw = 0)
        self.q_des = np.array([1, 0, 0, 0])

        # CG
        self.rCG = np.array([0.0, 0.0, 0.011])

        # Thrusters
        self.thruster_pos = [
            np.array([ 0.14, -0.092, 0.0]),
            np.array([ 0.14,  0.092, 0.0]),
            np.array([-0.14, -0.092, 0.0]),
            np.array([-0.14,  0.092, 0.0]),
            np.array([ 0.0, -0.109, 0.077]),
            np.array([ 0.0,  0.109, 0.077])
        ]

        self.thruster_dirs = [
            np.array([ 1, 0, 0]),
            np.array([ 1, 0, 0]),
            np.array([-1, 0, 0]),
            np.array([-1, 0, 0]),
            np.array([ 0, 0, 1]),
            np.array([ 0, 0, 1])
        ]

        self.B = self.compute_mixer()

        # PID yaw pitch roll
        self.pid_att = PID(kp=30, kd=10)

        # ROS
        self.create_subscription(Odometry,
                                 '/model/orca4/odometry',
                                 self.odom_cb,
                                 10)

        self.thruster_pubs = [
            self.create_publisher(Float64,
              f'/model/orca4/joint/thruster{i+1}_joint/cmd_thrust', 10)
            for i in range(6)
        ]

        self.timer = self.create_timer(0.01, self.control_loop)

    def compute_mixer(self):
        B = np.zeros((6, 6))
        for i in range(6):
            f = self.thruster_dirs[i]
            r = self.thruster_pos[i] - self.rCG
            B[0:3, i] = f
            B[3:6, i] = np.cross(r, f)
        return B

    def odom_cb(self, msg):
        o = msg.pose.pose.orientation
        self.q = np.array([o.w, o.x, o.y, o.z])
        self.omega = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])

    def control_loop(self):
        # Error de actitud (pequeños ángulos)
        error = -self.omega
        torque = self.pid_att.compute(error, self.omega)

        wrench = np.zeros(6)
        wrench[3:] = torque

        forces = np.linalg.pinv(self.B) @ wrench
        forces = np.clip(forces, -50, 50)

        for i, pub in enumerate(self.thruster_pubs):
            msg = Float64()
            msg.data = float(forces[i])
            pub.publish(msg)

def main():
    rclpy.init()
    node = Orca4Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

