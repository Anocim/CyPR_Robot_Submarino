import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import sys
import termios
import tty 
import math
import numpy as np

# Configuración de los Setpoints de Velocidad
LINEAR_VEL_STEP = 0.2    # m/s (incremento por pulsación)
ANGULAR_VEL_STEP = 0.2  # rad/s (incremento por pulsación)

# Configuración de los Setpoints de Posición/Orientación
DEPTH_STEP = 0.25      # metros (incremento por pulsación)
ANGLE_STEP_DEG = 5.0   # grados (por pulsación)
ANGLE_STEP_RAD = math.radians(ANGLE_STEP_DEG)

class TeleopFullSetpoint(Node):
    def __init__(self):
        super().__init__('teleop_full_setpoint_publisher')
        
        # Publicadores
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10) # Velocidad (Surge/Sway/Yaw Rate)
        self.pub_depth = self.create_publisher(Float64, '/target_depth', 10) # Profundidad
        self.pub_orientation = self.create_publisher(Twist, '/target_orientation_euler', 10) # Roll/Pitch/Yaw
        
        # Referencias/Setpoints de VELOCIDAD
        self.current_vel_setpoint = Twist()
        
        # Referencias/Setpoints de POSICIÓN/ORIENTACIÓN
        self.current_depth_setpoint = 0.0 
        self.current_roll_setpoint = 0.0 
        self.current_pitch_setpoint = 0.0
        self.current_yaw_setpoint = 0.0
        
        self.get_logger().info("--- Teleop de Referencias Integrado ---")
        self.get_logger().info("POSICIÓN/ORIENTACIÓN (Setpoints fijos):")
        self.get_logger().info("  Profundidad (Heave): 'k' (bajar), 'i' (subir)")
        self.get_logger().info("  Pitch: 'o' (cabeceo abajo), 'l' (cabeceo arriba)")
        self.get_logger().info("  Roll: 'u' (Roll izq), 'y' (Roll der)")
        self.get_logger().info("  Yaw Posición: 'j' (girar izq), 'ñ' (girar der)")
        self.get_logger().info("VELOCIDAD (Comando directo):")
        self.get_logger().info("  Surge (Vel X): 'w' (+), 's' (-)")
        self.get_logger().info("  Sway (Vel Y): 'a' (+), 'd' (-)")
        self.get_logger().info("  Yaw Rate (Vel Z): 'e' (+), 'q' (-)")
        self.get_logger().info("ESPACIO para resetear todas las referencias a cero.")
        self.get_logger().info("CTRL+C para salir.")
        
        self.publish_setpoints() # Publicar referencias iniciales

        self.read_keyboard()

    def get_key(self, settings):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
        return key

    def normalize_angle(self, angle):
        """Normaliza el ángulo al rango [-pi, pi]"""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def publish_setpoints(self):
        """Publica el estado actual de todas las referencias."""
        
        # 1. Publicar VELOCIDAD (Surge, Sway, Yaw Rate)
        self.pub_vel.publish(self.current_vel_setpoint)

        # 2. Publicar PROFUNDIDAD
        depth_msg = Float64()
        depth_msg.data = self.current_depth_setpoint
        self.pub_depth.publish(depth_msg)

        # 3. Publicar ORIENTACIÓN (Roll, Pitch, Yaw)
        orientation_msg = Twist()
        orientation_msg.angular.x = self.current_roll_setpoint
        orientation_msg.angular.y = self.current_pitch_setpoint
        orientation_msg.angular.z = self.current_yaw_setpoint
        self.pub_orientation.publish(orientation_msg)
        
        self.get_logger().info(
            f"Velocidad | X: {self.current_vel_setpoint.linear.x:.2f} | Y: {self.current_vel_setpoint.linear.y:.2f} | RZ: {self.current_vel_setpoint.angular.z:.2f}"
            f" | Orientación | Prof: {self.current_depth_setpoint:.2f}m | Yaw: {math.degrees(self.current_yaw_setpoint):.1f}°"
        )


    def read_keyboard(self):
        settings = termios.tcgetattr(sys.stdin)
        
        while rclpy.ok():
            key = self.get_key(settings)
            
            # --- MANEJO DE VELOCIDAD (Comando directo /cmd_vel) ---
            if key == 'w':
                self.current_vel_setpoint.linear.x += LINEAR_VEL_STEP # Surge adelante
            elif key == 's':
                self.current_vel_setpoint.linear.x -= LINEAR_VEL_STEP # Surge atrás
            elif key == 'd':
                self.current_vel_setpoint.linear.y -= LINEAR_VEL_STEP # Sway derecha
            elif key == 'a':
                self.current_vel_setpoint.linear.y += LINEAR_VEL_STEP # Sway izquierda
            elif key == 'e':
                self.current_vel_setpoint.angular.z -= ANGULAR_VEL_STEP # Yaw Rate derecha
            elif key == 'q':
                self.current_vel_setpoint.angular.z += ANGULAR_VEL_STEP # Yaw Rate izquierda

            # --- MANEJO de POSICIÓN/ORIENTACIÓN (Setpoints /target_...) ---
            elif key == 'i':
                self.current_depth_setpoint -= DEPTH_STEP # Subir
            elif key == 'k':
                self.current_depth_setpoint += DEPTH_STEP # Bajar
            elif key == 'o':
                self.current_pitch_setpoint -= ANGLE_STEP_RAD # Pitch abajo
            elif key == 'l':
                self.current_pitch_setpoint += ANGLE_STEP_RAD # Pitch arriba
            elif key == 'u':
                self.current_roll_setpoint += ANGLE_STEP_RAD # Roll izquierda
            elif key == 'y':
                self.current_roll_setpoint -= ANGLE_STEP_RAD # Roll derecha
            elif key == 'j':
                self.current_yaw_setpoint += ANGLE_STEP_RAD # Yaw (posición) izquierda
            elif key == 'ñ':
                self.current_yaw_setpoint -= ANGLE_STEP_RAD # Yaw (posición) derecha

            elif key == ' ':
                # Parar: todas las referencias a cero
                self.current_vel_setpoint = Twist()
                self.current_depth_setpoint = 0.0
                self.current_roll_setpoint = 0.0
                self.current_pitch_setpoint = 0.0
                self.current_yaw_setpoint = 0.0
            else:
                if (key == '\x03'): # CTRL+C
                    break
                continue
            
            # Normalizar ángulos
            self.current_roll_setpoint = self.normalize_angle(self.current_roll_setpoint)
            self.current_pitch_setpoint = self.normalize_angle(self.current_pitch_setpoint)
            self.current_yaw_setpoint = self.normalize_angle(self.current_yaw_setpoint)
            
            self.publish_setpoints()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopFullSetpoint()
    try:
        pass
    except Exception as e:
        node.get_logger().error(f"Excepción: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
