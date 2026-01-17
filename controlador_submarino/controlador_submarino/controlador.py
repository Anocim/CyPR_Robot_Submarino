#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray


# =========================
# MATRIZ DE ROTACIÓN RPY
# =========================
def rpy_matrix(roll, pitch, yaw):
    """
    Calcula la matriz de rotación a partir de ángulos de Euler (roll, pitch, yaw).
    Esta matriz convierte vectores del marco del motor al marco del cuerpo.
    
    Parámetros:
        roll, pitch, yaw : float
            Ángulos de rotación en radianes.
    
    Retorna:
        R : np.array (3x3)
            Matriz de rotación.
    """
    cr = np.cos(roll)
    sr = np.sin(roll)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cy = np.cos(yaw)
    sy = np.sin(yaw)

    return np.array([
        [cp*cy,  cr*sy - sr*sp*cy,  sr*sy + cr*sp*cy],
        [cp*sy, -cr*cy - sr*sp*sy,  sr*cy - cr*sp*sy],
        [-sp,    sr*cp,             cr*cp]
    ])


# =========================
# CONTROLADOR BAJO NIVEL
# =========================
class LowLevelController(Node):
    """
    Nodo ROS2 que implementa un controlador PD para la actitud de un vehículo.
    Calcula las fuerzas de los propulsores necesarias para mantener la orientación deseada.
    """

    def __init__(self):
        super().__init__('low_level_controller')

        # ---------- CENTRO DE GRAVEDAD ----------
        # Posición del centro de masa del vehículo en su marco de referencia
        self.r_CG = np.array([0.0, 0.0, 0.011])

        # ---------- DEFINICIÓN DE MOTORES ----------
        # Cada motor definido por:
        # - posición relativa al centro de gravedad
        # - orientación en ángulos RPY (roll, pitch, yaw)
        self.thrusters = [
            (np.array([0.14, -0.092, 0.0]), (-2, 2, -4)),
            (np.array([0.14,  0.092, 0.0]), (-2, 2, -12)),
            (np.array([0.14, -0.092, 0.0]), (-2, 2,  4)),
            (np.array([0.14,  0.092, 0.0]), (-2, 2, 12)),
            (np.array([0.0, -0.109, 0.077]), (0, 0, 0)),
            (np.array([0.0,  0.109, 0.077]), (0, 0, 0))
        ]

        # ---------- MATRIZ DE ASIGNACIÓN ----------
        # Relaciona las fuerzas y torques deseados con cada motor
        self.B = self.compute_allocation_matrix()

        # ---------- GANANCIAS CONTROL ----------
        # Matrices de ganancia para el controlador PD
        self.Kp = np.diag([5.0, 5.0, 1.0])   # ganancia proporcional
        self.Kd = np.diag([2.0, 2.0, 0.5])   # ganancia derivativa

        # ---------- ESTADOS ----------
        # Orientación actual en cuaternión [w, x, y, z]
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        # Velocidad angular actual [wx, wy, wz]
        self.omega = np.zeros(3)

        # ---------- SUSCRIPCIÓN ROS ----------
        # Suscribirse a datos de IMU
        self.sub_imu = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        # Publicador de comandos a los motores
        self.pub_thrusters = self.create_publisher(
            Float32MultiArray,
            '/thruster_cmds',
            10
        )

        self.get_logger().info("Low-level controller started")

    # =========================
    # MATRIZ DE ASIGNACIÓN B
    # =========================
    def compute_allocation_matrix(self):
        """
        Calcula la matriz de asignación B que relaciona fuerzas y torques deseados
        con cada propulsor. Esta matriz se usa para invertir la relación y obtener
        los comandos de los motores.
        
        Retorna:
            B : np.array (6xN_motores)
                Matriz de asignación de fuerzas y torques.
        """
        B = []

        for r_i, rpy in self.thrusters:
            R = rpy_matrix(*rpy)                  # Matriz de rotación del motor
            u_i = R @ np.array([0.0, 0.0, -1.0]) # Vector de empuje del motor
            tau_i = np.cross(r_i - self.r_CG, u_i) # Torque generado por el motor
            B.append(np.hstack((u_i, tau_i)))    # Concatenar fuerza y torque

        return np.array(B).T  # Devuelve la matriz 6xN

    # =========================
    # CALLBACK IMU
    # =========================
    def imu_callback(self, msg):
        """
        Se ejecuta al recibir un mensaje de IMU.
        Actualiza los estados y calcula las fuerzas de los motores.
        """
        # ---------- ACTUALIZAR ESTADO ----------
        self.q = np.array([
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ])

        self.omega = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        # ---------- ERROR DE ACTITUD ----------
        w, x, y, z = self.q
        e = np.sign(w) * np.array([x, y, z])

        # ---------- CONTROL PD ----------
        torque = self.Kp @ e - self.Kd @ self.omega

        # ---------- WRENCH DESEADO ----------
        wrench = np.zeros(6)
        wrench[3:] = torque  # solo se controla torque angular

        # ---------- INVERSA PARA MOTORES ----------
        thruster_forces = np.linalg.pinv(self.B) @ wrench

        # Normalización entre [-1, 1] considerando 100 N como máximo
        thruster_cmds = np.clip(thruster_forces / 100.0, -1.0, 1.0)

        # ---------- PUBLICAR COMANDOS ----------
        msg_out = Float32MultiArray()
        msg_out.data = thruster_cmds.tolist()
        self.pub_thrusters.publish(msg_out)


# =========================
# MAIN
# =========================
def main():
    """
    Inicializa el nodo ROS2, crea el controlador y mantiene el nodo vivo.
    """
    rclpy.init()
    node = LowLevelController()
    rclpy.spin(node)  # Mantiene el nodo en ejecución hasta que se cierre
    node.destroy_node()
    rclpy.shutdown()

