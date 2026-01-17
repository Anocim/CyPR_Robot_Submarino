import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

import numpy as np


class AUVController(Node):
    def __init__(self):
        super().__init__('control_bajo_nivel_auv')

        # --- MATRIZ DE ASIGNACIÓN ---
        self.T = np.array([
            [0.707,  0.707, -0.707, -0.707,  0.000,  0.000],
            [-0.707, 0.707, -0.707,  0.707,  0.000,  0.000],
            [0.000,  0.000,  0.000,  0.000, -1.000, -1.000],
            [0.000,  0.000,  0.000,  0.000, -0.109,  0.109],
            [0.000,  0.000,  0.000,  0.000,  0.000,  0.000],
            [0.160, -0.160, -0.160,  0.160,  0.000,  0.000]
        ])

        self.T_inv = np.linalg.pinv(self.T)

        # --- PARÁMETROS DEL CONTROLADOR PID ---
        self.dt = 0.05 
        
        # Ganancias Proporcionales (P)
        self.Kp = np.array([20, 20, 40, 10, 10, 15])
        # Ganancias Integrales (I) 
        self.Ki = np.array([0.5, 0.5, 5.0, 0.1, 0.1, 0.5])
        # Ganancias Derivativas (D)
        self.Kd = np.array([5, 5, 10, 2, 2, 3])

        # --- ESTADO Y ERROR INTEGRAL ---
        self.state_vel = np.zeros(6)    # [u v w p q r]
        self.target_vel = np.zeros(6)

        self.current_depth = 0.0
        self.target_depth = 0.5
        
        # Vector para almacenar el error acumulado
        self.error_integral = np.zeros(6)
        # Vector para almacenar el error en la última iteración
        self.last_error = np.zeros(6)


        # --- SUBSCRIPCIONES ---
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # --- PUBLISHERS ---
        self.thruster_topics = [
            '/model/orca4/joint/thruster1_joint/cmd_thrust',
            '/model/orca4/joint/thruster2_joint/cmd_thrust',
            '/model/orca4/joint/thruster3_joint/cmd_thrust',
            '/model/orca4/joint/thruster4_joint/cmd_thrust',
            '/model/orca4/joint/thruster5_joint/cmd_thrust',
            '/model/orca4/joint/thruster6_joint/cmd_thrust'
        ]

        self.thruster_pubs = []
        for topic in self.thruster_topics:
            self.thruster_pubs.append(
                self.create_publisher(Float64, topic, 10)
            )

        # --- LOOP DE CONTROL ---
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("Controlador AUV PID iniciado")

    # --------------------------------------------------

    def cmd_callback(self, msg):
        self.target_vel[0] = msg.linear.x
        self.target_vel[1] = msg.linear.y
        self.target_vel[5] = msg.angular.z

    def odom_callback(self, msg):
        self.state_vel[0] = msg.twist.twist.linear.x
        self.state_vel[1] = msg.twist.twist.linear.y
        self.state_vel[2] = msg.twist.twist.linear.z
        self.state_vel[3] = msg.twist.twist.angular.x
        self.state_vel[4] = msg.twist.twist.angular.y
        self.state_vel[5] = msg.twist.twist.angular.z

        self.current_depth = msg.pose.pose.position.z

    # --------------------------------------------------

    def control_loop(self):
        # 1. CÁLCULO DEL ERROR P (Proporcional)
        error = np.zeros(6)

        error[0] = self.target_vel[0] - self.state_vel[0]
        error[1] = self.target_vel[1] - self.state_vel[1]
        error[2] = self.target_depth - self.current_depth # Depth (control de posición)
        error[3] = 0.0 - self.state_vel[3]               # Roll (estabilizar a 0)
        error[4] = 0.0 - self.state_vel[4]               # Pitch (estabilizar a 0)
        error[5] = self.target_vel[5] - self.state_vel[5]

        # 2. CÁLCULO DEL ERROR I (Integral)
        # Acumulación: I = I_anterior + error * dt
        self.error_integral += error * self.dt
        
        I_limit = 10.0
        self.error_integral = np.clip(self.error_integral, -I_limit, I_limit)


        # 3. CÁLCULO DEL ERROR D (Derivativo)
        # tau_D = - Kd * state_vel

        # 4. CÁLCULO DEL TORQUE TOTAL (PID)
        # tau = Kp*error + Ki*integral_error + Kd*error_dot
        tau_P = self.Kp * error
        tau_I = self.Ki * self.error_integral
        tau_D = - self.Kd * self.state_vel # Usando la velocidad como derivada del error

        tau_total = tau_P + tau_I + tau_D

        # 5. Mixer
        motor_forces = self.T_inv @ tau_total
        motor_forces = np.clip(motor_forces, -200.0, 200.0)

        # 6. Publicar a cada thruster
        for i in range(6):
            msg = Float64()
            msg.data = float(motor_forces[i])
            self.thruster_pubs[i].publish(msg)

        self.last_error = error


def main(args=None):
    rclpy.init(args=args)
    node = AUVController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
