import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import FluidPressure, Imu

from tf_transformations import euler_from_quaternion

import numpy as np
import math

from sensor_msgs.msg import FluidPressure
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan

class AUVController(Node):
    def __init__(self):
        super().__init__('control_dinamica_pid_auv')

        qos_profile = QoSProfile(
           reliability=ReliabilityPolicy.RELIABLE,
           durability=DurabilityPolicy.VOLATILE,
           history=HistoryPolicy.KEEP_LAST,
           depth=10
        )

        # --- PARÁMETROS FÍSICOS (DINÁMICA) ---
        # Masa e Inercias (incluye inercia de masa añadida si el modelo fuera completo)
        self.m = 10.0
        self.Ixx = 0.09873097998042396
        self.Iyy = 0.17756847998042397
        self.Izz = 0.2692441666666667
        
        # Coeficientes de arrastre D(v) [u, v, w, p, q, r]
        self.d_abs = np.array([33.8, 54.26875, 73.37135, 4.0, 4.0, 4.0])
        
        # Parámetros de Restitución
        self.W = self.m * 9.81
        self.B = 98.4068565555  
        self.CoG = np.array([0.0, 0.0, 0.011])
        self.CoB = np.array([0.0, 0.0, 0.03253])

        # --- MATRIZ DE ASIGNACIÓN (MIXER) ---
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
        
        # 1. Parámetros de Inercia Mii (Solo cuerpo rígido, falta masa añadida)
        # Asumiendo que M_total = M_rigida + M_añadida
        # Aquí, usaremos la masa rígida (m=10) o la inercia (Izz) como aproximación
        M_ii = np.array([self.m, self.m, self.m, self.Ixx, self.Iyy, self.Izz])
        
        # 2. Parámetro de Diseño de Control (Frecuencia Natural)
        # Queremos un tiempo de establecimiento rápido. Una wn de 2.0 es rápida (Ts ~ 4/wn = 2s)
        self.wn = 2.0 
        # Queremos amortiguamiento crítico (sin sobreimpulso)
        self.zeta = 1.0 

        # CÁLCULO ANALÍTICO DE GANANCIAS (para ejes de velocidad u, v, r)
        self.Kp = np.zeros(6)
        self.Kd = np.zeros(6)
        
        for i in range(6):
            # Asignación de Polos para un sistema de 2do orden:
            # Kp = M_ii * wn^2
            # Kd = M_ii * 2 * zeta * wn
            # Usaremos solo Mii para el cálculo, asumiendo que el Damping (D(v)v) está compensado
            self.Kp[i] = M_ii[i] * (self.wn ** 2)
            self.Kd[i] = M_ii[i] * 2.0 * self.zeta * self.wn
            
        # El eje 2 (Heave/Profundidad) es control de POSICIÓN (no velocidad), Kp y Ki dominan. 
        # Lo ajustamos manualmente (o con un diseño específico de 3er orden)

        # Ganancias Integrales (Ki): Necesitan ser ajustadas empíricamente 
        # ya que compensan errores de estado estacionario y flotabilidad no compensados.
        self.Ki = np.array([10., 10., 10., 1.1, 1.1, 0.5]) 
        
        # --- ESTADO Y ERROR INTEGRAL ---
        self.state_vel = np.zeros(6)
        self.target_vel = np.zeros(6)

        self.current_depth = 0.0
        self.target_depth = 0.0
        
        self.current_phi = 0.0
        self.current_theta = 0.0
        self.current_psi = 0.0
        
        self.error_integral = np.zeros(6)

        # --- SUBSCRIPCIONES Y PUBLICADORES ---
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        self.create_subscription(Odometry, '/model/orca4/odometry', self.odom_callback, qos_profile)

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
                self.create_publisher(Float64, topic, qos_profile)
            )

        # --- LOOP DE CONTROL ---
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(f"Controlador PID (Analítico) con Compensación Dinámica iniciado.")
        self.get_logger().info(f"Ganancias Kp calculadas: {self.Kp}")
        self.get_logger().info(f"Ganancias Kd calculadas: {self.Kd}")


    # --------------------------------------------------
    # --- FUNCIONES DINÁMICAS (sin cambios) ---
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
        
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        (self.current_phi, self.current_theta, self.current_psi) = euler_from_quaternion(quat)

    def calculate_coriolis_term(self, nu):
        """Calcula C_RB(nu)*nu."""
        m = self.m
        Ixx, Iyy, Izz = self.Ixx, self.Iyy, self.Izz
        u, v, w, p, q, r = nu

        tau_coriolis = np.zeros(6)
        
        # Fuerzas lineales
        tau_coriolis[0] = m * (q*w - r*v)
        tau_coriolis[1] = m * (r*u - p*w)
        tau_coriolis[2] = m * (p*v - q*u)

        # Torques angulares
        tau_coriolis[3] = (Izz - Iyy) * q * r
        tau_coriolis[4] = (Ixx - Izz) * p * r
        tau_coriolis[5] = (Iyy - Ixx) * p * q
        
        return tau_coriolis

    def calculate_damping_term(self, nu):
        """Calcula D(nu)*nu (Arrastre de segundo orden)."""
        nu_abs = np.abs(nu)
        D_diag_v_abs = self.d_abs * nu_abs
        
        return -D_diag_v_abs * nu

    def calculate_restoring_term(self):
        """Calcula g(eta) (Gravedad y Flotación)."""
        W, B = self.W, self.B
        xg, yg, zg = self.CoG
        xb, yb, zb = self.CoB
        phi, theta = self.current_phi, self.current_theta 

        g_eta = np.zeros(6)

        g_eta[2] = -(W - B) 

        cphi = math.cos(phi)
        sphi = math.sin(phi)
        ctheta = math.cos(theta)
        stheta = math.sin(theta)
        
        g_eta[3] = (yg*W - yb*B) * ctheta * cphi + (zg*W - zb*B) * stheta * sphi
        g_eta[4] = (zg*W - zb*B) * ctheta * sphi + (xg*W - xb*B) * stheta * cphi
        g_eta[5] = -(xg*W - xb*B) * stheta + (yg*W - yb*B) * ctheta * sphi
        
        return g_eta


    # --------------------------------------------------
    # --- BUCLE PRINCIPAL (sin cambios en la lógica PID/FF) ---
    # --------------------------------------------------

    def control_loop(self):
        
        # 1. CÁLCULO DEL ERROR P (Feedback)
        error = np.zeros(6)
        error[0] = self.target_vel[0] - self.state_vel[0]
        error[1] = self.target_vel[1] - self.state_vel[1]
        error[2] = self.target_depth - self.current_depth 
        error[3] = 0.0 - self.current_phi
        error[4] = 0.0 - self.current_theta 
        error[5] = self.current_psi - self.current_psi 

        # 2. CÁLCULO DEL ERROR I (Integral)
        self.error_integral += error * self.dt
        
        I_limit = 10.0
        self.error_integral = np.clip(self.error_integral, -I_limit, I_limit)

        # 3. FUERZA DE FEEDBACK (PID)
        tau_P = self.Kp * error
        tau_I = self.Ki * self.error_integral
        tau_D = - self.Kd * self.state_vel 
        
        #tau_feedback = tau_P + tau_I + tau_D
        tau_feedback = tau_P + tau_I + tau_D

        # 4. FUERZA DE FEEDFORWARD (Compensación Dinámica)
        tau_coriolis = self.calculate_coriolis_term(self.state_vel) #despreciable a baja velocidad y dificil de modelar
        tau_damping = self.calculate_damping_term(self.state_vel) #despreciable y se compensa con el control derivativo
        tau_restoring = self.calculate_restoring_term()

        #tau_feedforward = tau_coriolis + tau_damping + tau_restoring
        tau_feedforward = tau_restoring
        
        # 5. FUERZA DE CONTROL TOTAL
        tau_total = tau_feedback + tau_feedforward
        
        # 6. MIXER y Publicación
        motor_forces = self.T_inv @ tau_total
        motor_forces = np.clip(motor_forces, -100.0, 100.0)

        for i in range(6):
            msg = Float64()
            msg.data = float(motor_forces[i])
            self.thruster_pubs[i].publish(msg)


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
