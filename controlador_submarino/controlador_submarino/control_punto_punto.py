import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import math

class NodoGuiado(Node):
    def __init__(self):
        super().__init__('referencia_cambio')

        # --- 1. PUBLICADORES ---
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        # Enviamos profundidad (Z)
        self.pub_depth = self.create_publisher(Float64, '/target_depth', 10)
        
        # --- 2. SUSCRIPTORES (Desde el simulador) ---
        self.create_subscription(Odometry, '/model/orca4/odometry', self.odom_callback, 10)

        # --- 3. CONFIGURACIÓN DE LA MISIÓN ---
        self.waypoints = [
            (10.0, 10.0), 
            (20.0, 10.0), 
            (20.0, 20.0),
            (0.0, 0.0)
        ]
        self.profundidad_operativa = -7.0
        self.indice_wp = 0
        
        # Estado del robot
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        # Estado de la misión
        self.fase = "INMERSION" # INMERSION -> NAVEGACION -> ASCENSO -> FIN
        
        # Loop de control (0.1s = 10Hz)
        self.timer = self.create_timer(0.1, self.control_logic)
        self.get_logger().info('--- NODO DE GUIADO INICIADO ---')

    def odom_callback(self, msg):
        """Actualiza la posición real del robot"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z

    def control_logic(self):
        if self.fase == "FIN":
            return

        msg_vel = Twist()
        msg_depth = Float64()
        
        # --- LÓGICA DE FASES ---
        
        if self.fase == "INMERSION":
            # Objetivo: Bajar a -7m, quietos en X,Y
            msg_depth.data = self.profundidad_operativa
            msg_vel.linear.x = 0.0
            msg_vel.linear.y = 0.0
            
            # Condición de cambio: Estamos cerca de -7m?
            if abs(self.current_z - self.profundidad_operativa) < 0.5:
                self.get_logger().info("Profundidad alcanzada. Iniciando NAVEGACION.")
                self.fase = "NAVEGACION"

        elif self.fase == "NAVEGACION":
            # Mantenemos profundidad
            msg_depth.data = self.profundidad_operativa
            
            # Obtenemos el waypoint actual
            target_x, target_y = self.waypoints[self.indice_wp]
            
            # Calcular vector hacia el objetivo
            error_x = target_x - self.current_x
            error_y = target_y - self.current_y
            distancia = math.sqrt(error_x**2 + error_y**2)
            
            if distancia < 1.0: # Radio de aceptación de 1 metro
                self.get_logger().info(f"Waypoint {self.indice_wp+1} alcanzado.")
                self.indice_wp += 1
                
                # Si se acabaron los puntos, pasamos a ascenso
                if self.indice_wp >= len(self.waypoints):
                    self.fase = "ASCENSO"
            else:
                # CONTROLADOR P (Proporcional) SIMPLE PARA VELOCIDAD
                kp = 0.5 
                vel_x = error_x * kp
                vel_y = error_y * kp
                
                vel_max = 1.5
                vel_x = max(min(vel_x, vel_max), -vel_max)
                vel_y = max(min(vel_y, vel_max), -vel_max)
                
                msg_vel.linear.x = vel_x
                msg_vel.linear.y = vel_y

        elif self.fase == "ASCENSO":
            msg_depth.data = 0.0 # Superficie
            msg_vel.linear.x = 0.0
            msg_vel.linear.y = 0.0
            
            if abs(self.current_z) < 0.5:
                self.get_logger().info("Misión Completada. Robot en superficie.")
                self.fase = "FIN"

        # --- PUBLICAR COMANDOS AL SDRE ---
        self.pub_vel.publish(msg_vel)
        self.pub_depth.publish(msg_depth)

def main(args=None):
    rclpy.init(args=args)
    node = NodoGuiado()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()