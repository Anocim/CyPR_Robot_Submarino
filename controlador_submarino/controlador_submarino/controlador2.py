#!/usr/bin/env python3
import numpy as np

# =========================
# CONFIGURACIÓN DEL ROBOT
# =========================
rCG = np.array([0.0, 0.0, 0.011])  # Centro de gravedad
# Vectores de dirección de cada motor (todos hacia abajo)
u = [np.array([0, 0, -1]) for _ in range(6)]
# Posición y orientación de los motores: p = [x, y, z, roll, pitch, yaw]
p = [
    np.array([0.14, -0.092, 0, -2, 2, -4]),
    np.array([0.14,  0.092, 0, -2, 2, -12]),
    np.array([0.14, -0.092, 0, -2, 2, 4]),
    np.array([0.14,  0.092, 0, -2, 2, 12]),
    np.array([0.0, -0.109, 0.077, 0, 0, 0]),
    np.array([0.0,  0.109, 0.077, 0, 0, 0])
]

# =========================
# FUNCIONES AUXILIARES
# =========================
def quaternion_inverse(q):
    """Inversa de un cuaternión"""
    w, x, y, z = q
    return np.array([w, -x, -y, -z]) / np.dot(q, q)

def quaternion_multiply(q1, q2):
    """Multiplicación de cuaterniones"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return np.array([w, x, y, z])

def sign_quaternion(q):
    """Error de actitud vectorial"""
    w, x, y, z = q
    return np.array([x, y, z]) * np.sign(w)

def rotation_matrix_from_rpy(roll, pitch, yaw):
    """Matriz de rotación a partir de roll-pitch-yaw en grados"""
    r, p, y = np.deg2rad([roll, pitch, yaw])
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(r), -np.sin(r)],
                   [0, np.sin(r),  np.cos(r)]])
    Ry = np.array([[ np.cos(p), 0, np.sin(p)],
                   [0, 1, 0],
                   [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0],
                   [np.sin(y),  np.cos(y), 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx

# =========================
# MIXER: Fuerza+Torque -> Motores
# =========================
def compute_mixer(u_vectors, p_vectors, rCG):
    """
    Construye la matriz de control (6x6)
    """
    B = np.zeros((6, 6))
    for i in range(6):
        # Fuerza lineal
        B[:3, i] = u_vectors[i]
        # Torque generado = (ri - rCG) x fi*ui
        r_i = p_vectors[i][:3]
        B[3:, i] = np.cross(r_i - rCG, u_vectors[i])
    return B

def mixer(wrench_desired, B):
    """
    Calcula fuerzas de motor a partir de wrench deseado
    """
    # Inversa de la matriz B (asumimos cuadrada)
    B_inv = np.linalg.inv(B)
    motor_forces = B_inv @ wrench_desired
    # Saturación entre -100 y 100 N
    motor_forces = np.clip(motor_forces, -100, 100)
    return motor_forces

# =========================
# CONTROL PID
# =========================
class PID:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = np.zeros(3)
        self.prev_error = np.zeros(3)
    
    def compute(self, error, derivative_measure=np.zeros(3)):
        self.integral += error * self.dt
        derivative = derivative_measure
        output = self.kp*error + self.ki*self.integral - self.kd*derivative
        self.prev_error = error
        return output

# =========================
# CONTROLADOR PRINCIPAL
# =========================
class LowLevelController:
    def __init__(self, dt):
        self.dt = dt
        # PID actitud
        self.pid_attitude = PID(kp=50, ki=0.0, kd=20, dt=dt)
        # PID traslación
        self.pid_position = PID(kp=30, ki=0.0, kd=10, dt=dt)
        # Matriz del mixer
        self.B = compute_mixer(u, p, rCG)
    
    def compute_wrench(self, q_actual, q_desired, omega, vel_lin, vel_des=np.zeros(3)):
        """
        Calcula wrench deseado [Fx, Fy, Fz, Tx, Ty, Tz]
        """
        # --- Actitud ---
        qe = quaternion_multiply(q_desired, quaternion_inverse(q_actual))
        e_att = sign_quaternion(qe)
        torque = self.pid_attitude.compute(e_att, omega)
        
        # --- Traslación ---
        error_pos = vel_des - vel_lin
        force = self.pid_position.compute(error_pos)
        
        # Concatenamos wrench total
        wrench = np.zeros(6)
        wrench[:3] = force
        wrench[3:] = torque
        return wrench
    
    def compute_motor_commands(self, q_actual, q_desired, omega, vel_lin, vel_des=np.zeros(3)):
        wrench = self.compute_wrench(q_actual, q_desired, omega, vel_lin, vel_des)
        motor_forces = mixer(wrench, self.B)
        return motor_forces

# =========================
# EJEMPLO DE USO
# =========================
if __name__ == "__main__":
    dt = 0.01  # 100 Hz
    controller = LowLevelController(dt=dt)

    # Cuaternión actual y deseado (ejemplo)
    q_actual = np.array([1, 0, 0, 0])
    q_desired = np.array([1, 0, 0, 0])
    omega = np.zeros(3)  # Velocidad angular
    vel_lin = np.zeros(3)  # Velocidad lineal
    vel_des = np.array([0.5, 0.0, 0.0])  # Movimiento deseado hacia adelante

    motor_cmds = controller.compute_motor_commands(q_actual, q_desired, omega, vel_lin, vel_des)
    print("Fuerzas de motores:", motor_cmds)

