import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

import numpy as np


class AUVController(Node):
    def __init__(self):
        super().__init__('control_bajo_nivel_auv')

        # --- PARÁMETROS FÍSICOS (extraídos del modelo SDF) ---
        # [cite_start]Masa y momentos de inercia [cite: 391-404]
        self.m = 10.0
        self.Ixx = 0.09873097998042396
        self.Iyy = 0.17756847998042397
        self.Izz = 0.2692441666666667
        
        # [cite_start]Coeficientes de arrastre (segundo orden, D(v)) [cite: 515-520, 533-538]
        # [u, v, w, p, q, r]
        self.d_abs = np.array([33.8, 54.26875, 73.37135, 4.0, 4.0, 4.0])
        
        # [cite_start]Parámetros de Restitución [cite: 553-567]
        self.W = self.m * 9.81  # Peso (mg)
        self.B = 98.4068565555  # Flotabilidad (Asumiendo que g(eta)[2] = -0.3 N)
        self.CoG = np.array([0.0, 0.0, 0.011])  # Centro de Gravedad
        self.CoB = np.array([0.0, 0.0, 0.03253]) # Centro de Flotación

        # --- MATRIZ DE ASIGNACIÓN (MIXER) ---
        # Mapea fuerzas/torques (6DOF) a fuerzas de motor (6 thrusters)
        self.T = np.array([
            [0.707,  0.707, -0.707, -0.707,  0.000,  0.000],
            [-0.707, 0.707, -0.707,  0.707,  0.000,  0.000],
            [0.000,  0.000,  0.000,  0.000, -1.000, -1.000],
            [0.000,  0.000,  0.000,  0.000, -0.109,  0.109],
            [0.000,  0.000,  0.000,  0.000,  0.000,  0.000],
            [0.160, -0.160, -0.160,  0.160,  0.000,  0.000]
        ])

        # Se usa la pseudo-inversa para el mixer
        self.T_inv = np.linalg.pinv(self.T)

        # --- GANANCIAS DEL CONTROLADOR PD ---
        self.Kp = np.array([20, 20, 40, 10, 10, 15])
        self.Kd = np.array([5, 5, 10, 2, 2, 3])

        # --- ESTADO Y REFERENCIAS ---
        self.state_vel = np.zeros(6)    # Velocidad actual [u v w p q r]
        self.target_vel = np.zeros(6)   # Velocidad de referencia
        
        self.current_depth = 0.0        # Profundidad actual
        self.target_depth = 0.5         # Profundidad de referencia
        
        # Asumimos que los ángulos de Roll y Pitch (para g(eta)) son 0 en bajo nivel
        self.current_phi = 0.0
        self.current_theta = 0.0
        

        # --- SUBSCRIPCIONES ---
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # --- PUBLISHERS (Thrusters) ---
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
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info("Controlador AUV con compensación dinámica iniciado")

    # --------------------------------------------------
    # --- FUNCIONES DINÁMICAS ---
    # --------------------------------------------------

    def calculate_coriolis_term(self, nu):
        """
        Calcula C_RB(nu)*nu. Solo para cuerpo rígido.
        nu = [u, v, w, p, q, r]
        """
        m = self.m
        Ixx, Iyy, Izz = self.Ixx, self.Iyy, self.Izz
        u, v, w, p, q, r = nu

        tau_coriolis = np.zeros(6)
        
        # [cite_start]Fuerzas lineales: m * (omega x v) [cite: 460-467]
        tau_coriolis[0] = m * (q*w - r*v)
        tau_coriolis[1] = m * (r*u - p*w)
        tau_coriolis[2] = m * (p*v - q*u)

        # [cite_start]Torques angulares: - (I * omega) x omega [cite: 482-488]
        tau_coriolis[3] = (Izz - Iyy) * q * r
        tau_coriolis[4] = (Ixx - Izz) * p * r
        tau_coriolis[5] = (Iyy - Ixx) * p * q
        
        return tau_coriolis

    def calculate_damping_term(self, nu):
        """
        Calcula D(nu)*nu (Arrastre de segundo orden).
        D(v) = -diag(d_i * |vi|) * v
        """
        [cite_start]# [cite: 526, 529-540]
        nu_abs = np.abs(nu)
        D_diag_v_abs = self.d_abs * nu_abs
        
        # Fh = - (d_i * |v_i|) * v_i
        return -D_diag_v_abs * nu

    def calculate_restoring_term(self):
        """
        Calcula g(eta) (Gravedad y Flotación). 
        Se asume phi y theta = 0 para el cálculo de tau_g para este bajo nivel simple.
        """
        W, B = self.W, self.B
        xg, yg, zg = self.CoG
        xb, yb, zb = self.CoB
        phi, theta = self.current_phi, self.current_theta # Ángulos Roll y Pitch

        g_eta = np.zeros(6)

        # Fuerza en Z (Heave): W - B
        # [cite_start]Usamos el valor implícito del documento: W - B = -0.3 N [cite: 571]
        g_eta[2] = -(W - B) 

        # Torques de Roll (K), Pitch (M) y Yaw (N)
        # Se necesita la matriz de rotación para un cálculo completo, pero simplificando:
        
        cphi = np.cos(phi)
        sphi = np.sin(phi)
        ctheta = np.cos(theta)
        stheta = np.sin(theta)
        
        # Torque K (Roll)
        g_eta[3] = (yg*W - yb*B) * ctheta * cphi + (zg*W - zb*B) * stheta * sphi
        
        # Torque M (Pitch)
        g_eta[4] = (zg*W - zb*B) * ctheta * sphi + (xg*W - xb*B) * stheta * cphi
        
        # Torque N (Yaw)
        g_eta[5] = -(xg*W - xb*B) * stheta + (yg*W - yb*B) * ctheta * sphi

        # Nota: Simplificando roll y pitch pequeños (cphi=1, sphi=0, ctheta=1, stheta=0):
        # g_eta[3] = (yg*W - yb*B) * 1 * 1 + (zg*W - zb*B) * 0 * 0 = 0 (si yg=yb, zg=zb)
        # [cite_start]El documento simplifica el vector a [0, 0, -0.3, 0, 0, 0] [cite: 571]
        
        return g_eta


    # --------------------------------------------------
    # --- CALLBACKS Y BUCLE PRINCIPAL ---
    # --------------------------------------------------

    def cmd_callback(self, msg):
        """Recibe la velocidad deseada /cmd_vel"""
        self.target_vel[0] = msg.linear.x    # Surge (u)
        self.target_vel[1] = msg.linear.y    # Sway (v)
        self.target_vel[5] = msg.angular.z   # Yaw (r)

    def odom_callback(self, msg):
        """Recibe el estado actual /odom"""
        # [cite_start]Velocidad (marco del cuerpo) [cite: 34, 35]
        self.state_vel[0] = msg.twist.twist.linear.x    # u
        self.state_vel[1] = msg.twist.twist.linear.y    # v
        self.state_vel[2] = msg.twist.twist.linear.z    # w
        self.state_vel[3] = msg.twist.twist.angular.x   # p
        self.state_vel[4] = msg.twist.twist.angular.y   # q
        self.state_vel[5] = msg.twist.twist.angular.z   # r

        # Posición (profundidad, Z en marco NED)
        self.current_depth = msg.pose.pose.position.z
        
        # Extracción de ángulos de Euler (necesario para g(eta) si se hiciera completo)
        # Los cuaterniones están en msg.pose.pose.orientation
        # Se requiere una conversión de cuaternión a RPY para phi/theta

    def control_loop(self):
        """Bucle de control principal (PD + Feedforward dinámico)"""
        
        # 1. CÁLCULO DEL ERROR (PD)
        error = np.zeros(6)

        # Traslación (Velocidad en X, Y, y Posición en Z/Profundidad)
        error[0] = self.target_vel[0] - self.state_vel[0]    # Surge
        error[1] = self.target_vel[1] - self.state_vel[1]    # Sway
        error[2] = self.target_depth - self.current_depth    # Depth (Profundidad)
        
        # Rotación (Señal de velocidad de giro para Roll, Pitch, Yaw)
        # Se busca estabilizar Roll y Pitch a 0, y seguir el Yaw deseado
        error[3] = 0.0 - self.state_vel[3]                   # Roll (p)
        error[4] = 0.0 - self.state_vel[4]                   # Pitch (q)
        error[5] = self.target_vel[5] - self.state_vel[5]    # Yaw (r)

        # 2. CÁLCULO DE LA SALIDA DEL PD (Fuerza para corregir errores)
        # tau_PD = Kp * error + Kd * error_dot (usamos -Kd * state_vel, asumiendo error_dot = -state_vel)
        tau_pd = self.Kp * error - self.Kd * self.state_vel

        # 3. CÁLCULO DE LA COMPENSACIÓN DINÁMICA (Feedforward)
        # tau_dynamics = C(nu)nu + D(nu)nu + g(eta)
        
        tau_coriolis = self.calculate_coriolis_term(self.state_vel)
        tau_damping = self.calculate_damping_term(self.state_vel)
        tau_restoring = self.calculate_restoring_term()

        tau_dynamics = tau_coriolis + tau_damping + tau_restoring
        
        # 4. FUERZA DE CONTROL TOTAL (tau_total)
        # tau_total es el vector de fuerzas y torques 6DOF deseados
        tau_total = tau_pd + tau_dynamics
        
        # 5. MIXER: Asignación a los propulsores
        # [cite_start]motor_forces = T_inv @ tau_total [cite: 313-315]
        motor_forces = self.T_inv @ tau_total
        
        # [cite_start]Saturación de fuerzas (se asume un límite de +/- 200 N, ya que cada propulsor es +/- 100 N) [cite: 574]
        motor_forces = np.clip(motor_forces, -200.0, 200.0)

        # 6. PUBLICAR
        for i in range(6):
            msg = Float64()
            msg.data = float(motor_forces[i])
            self.thruster_pubs[i].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AUVController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
