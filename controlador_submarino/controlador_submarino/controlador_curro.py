import rclpy
from rclpy.node import Node
# Mensajes estándar de ROS
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np
# Librería para manejar rotaciones fácilmente (cuaterniones a ángulos)
from scipy.spatial.transform import Rotation as R


class AUVController(Node):
    def __init__(self):
        super().__init__('control_bajo_nivel_auv')

        # --- CONFIGURACIÓN ---
        # 1. Matriz de Asignación (La que calculamos arriba)
        # Relaciona: [Surge, Sway, Heave, Roll, Pitch, Yaw] -> [Motor1 ... Motor6]
        self.T = np.array([
            [0.707,  0.707, -0.707, -0.707,  0.000,  0.000],  # Surge (X)
            [-0.707,  0.707, -0.707,  0.707,  0.000,  0.000],  # Sway (Y)
            [0.000,  0.000,  0.000,  0.000, -1.000, -1.000],  # Heave (Z)
            [0.000,  0.000,  0.000,  0.000, -0.109,  0.109],  # Roll
            [0.000,  0.000,  0.000,  0.000,  0.000,
                0.000],  # Pitch (Simplificado 0)
            [0.160, -0.160, -0.160,  0.160,  0.000,  0.000]  # Yaw (Giro)
        ])

        # Calculamos la INVERSA (Pseudo-inversa) una sola vez.
        # Esto nos permite hacer la operación contraria: Fuerza Deseada -> Motores
        self.T_inv = np.linalg.pinv(self.T)

        # 2. Ganancias del PID (Esto es el "tacto" del piloto)
        # Kp alto = Reacción brusca. Kp bajo = Reacción lenta.
        # [x, y, z, roll, pitch, yaw]
        self.Kp = np.array([20.0, 20.0, 40.0, 10.0, 10.0, 15.0])
        # Frenado (Derivativo)
        self.Kd = np.array([5.0,  5.0, 10.0,  2.0,  2.0,  3.0])

        # --- COMUNICACIÓN ROS ---
        # Recibir objetivo de velocidad (ej: del joystick o planificador)
        self.sub_cmd = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)
        # Recibir estado real del robot (del simulador/sensores)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        # Enviar órdenes a los motores
        self.pub_thrusters = self.create_publisher(
            Float64MultiArray, '/thrusters/cmd', 10)

        # Variables para guardar datos
        self.state_vel = np.zeros(6)       # Velocidad actual [u,v,w,p,q,r]
        self.target_vel = np.zeros(6)      # Velocidad deseada
        self.current_depth = 0.0           # Profundidad actual
        # Profundidad objetivo (ej: 0.5 metros)
        self.target_depth = 0.5

        # Bucle de control: Se ejecuta 20 veces por segundo (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

    def cmd_callback(self, msg):
        # Guardamos lo que nos piden
        self.target_vel[0] = msg.linear.x  # Surge
        self.target_vel[1] = msg.linear.y  # Sway
        self.target_vel[5] = msg.angular.z  # Yaw rate
        # Nota: Normalmente la Z del joystick se usa para subir/bajar profundidad objetivo
        # Si msg.linear.z es != 0, podríamos cambiar self.target_depth

    def odom_callback(self, msg):
        # 1. Leer Velocidad Lineal y Angular
        # OJO: Gazebo a veces da la velocidad en el mundo, hay que asegurarse que sea en el cuerpo.
        # Asumimos que viene en el marco del cuerpo (Body Frame) para simplificar.
        self.state_vel[0] = msg.twist.twist.linear.x
        self.state_vel[1] = msg.twist.twist.linear.y
        self.state_vel[2] = msg.twist.twist.linear.z
        self.state_vel[3] = msg.twist.twist.angular.x
        self.state_vel[4] = msg.twist.twist.angular.y
        self.state_vel[5] = msg.twist.twist.angular.z

        # 2. Leer Profundidad (Posición Z)
        self.current_depth = msg.pose.pose.position.z

    def control_loop(self):
        # --- PASO 1: CALCULAR EL ERROR ---
        # Error = Lo que quiero - Lo que tengo
        error = np.zeros(6)

        # Velocidades lineales (Surge, Sway)
        error[0] = self.target_vel[0] - self.state_vel[0]
        error[1] = self.target_vel[1] - self.state_vel[1]

        # Profundidad (Heave): Aquí controlamos POSICIÓN, no velocidad
        # Queremos mantenernos en target_depth
        error[2] = (self.target_depth - self.current_depth)

        # Estabilidad (Roll, Pitch): Queremos que sean 0 (robot plano)
        # Asumimos que leemos la orientación (aquí simplificado, idealmente usarías cuaterniones)
        error[3] = 0.0 - self.state_vel[3]  # Queremos velocidad de roll 0
        error[4] = 0.0 - self.state_vel[4]  # Queremos velocidad de pitch 0

        # Giro (Yaw): Controlamos velocidad de giro
        error[5] = self.target_vel[5] - self.state_vel[5]

        # --- PASO 2: CONTROLADOR PID ---
        # Calculamos la Fuerza/Torque total necesaria (Vector Tau)
        # Tau = Error * Kp - Velocidad * Kd
        tau = error * self.Kp

        # --- PASO 3: MIXER (Asignación de Motores) ---
        # ¡Aquí usamos la matriz que calculamos!
        # Motores = T_inversa * Tau
        motor_forces = np.dot(self.T_inv, tau)

        # Limitar fuerzas (el motor no da infinito)
        motor_forces = np.clip(motor_forces, -100, 100)  # Entre -100N y 100N

        # --- PASO 4: PUBLICAR ---
        msg = Float64MultiArray()
        msg.data = motor_forces.tolist()
        self.pub_thrusters.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AUVController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
