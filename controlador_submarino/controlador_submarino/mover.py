import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class MoverHaciaDelante(Node):
    def __init__(self):
        super().__init__('mover_adelante')
        
        # --- QoS (Igual que en el mapeo) ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Definimos los 6 motores
        self.topic_names = [
            '/model/orca4/joint/thruster1_joint/cmd_thrust', # Horizontal Delantero Dcha
            '/model/orca4/joint/thruster2_joint/cmd_thrust', # Horizontal Delantero Izq
            '/model/orca4/joint/thruster3_joint/cmd_thrust', # Horizontal Trasero Dcha
            '/model/orca4/joint/thruster4_joint/cmd_thrust', # Horizontal Trasero Izq
            '/model/orca4/joint/thruster5_joint/cmd_thrust', # Vertical
            '/model/orca4/joint/thruster6_joint/cmd_thrust'  # Vertical
        ]

        self.pubs = []
        for name in self.topic_names:
            self.pubs.append(self.create_publisher(Float64, name, qos_profile))

        self.get_logger().info("--- AVANZANDO ---")
        self.get_logger().info("Activando motores 1, 2, 3 y 4.")

        # Un solo timer que manda la orden constantemente (20Hz)
        self.timer = self.create_timer(0.05, self.enviar_fuerza_avance)

    def enviar_fuerza_avance(self):
        # FUERZA: Ajusta este valor. 
        # Positivo suele ser empujar. Si va marcha atrás, ponlo negativo (-80.0)
        msg_pos = Float64()
        msg_pos.data = 120.0
        
        msg_neg = Float64()
        msg_neg.data = 0.0  # Fuerza inversa
        
        fuerza_vertical =100.0  # Ponemos 300N para vencer la flotabilidad
        
        msg_vertical = Float64()
        msg_vertical.data = fuerza_vertical
        # --- MOTORES HORIZONTALES (1, 2, 3, 4) ---
        # Para ir recto hacia adelante, activamos los 4 en configuración vectorial
        self.pubs[0].publish(msg_pos) # Motor 1
        self.pubs[1].publish(msg_neg) # Motor 2
        self.pubs[2].publish(msg_neg) # Motor 3
        self.pubs[3].publish(msg_pos) # Motor 4

        # --- MOTORES VERTICALES (5, 6) ---
        # Los dejamos a 0 (o quietos) para que no suba ni baje
        msg_stop = Float64()
        msg_stop.data = 0.0
        self.pubs[4].publish(msg_vertical)
        self.pubs[5].publish(msg_vertical)

    def stop_all(self):
        """Función de seguridad para parar todo al cerrar"""
        self.get_logger().info("Parando motores...")
        msg = Float64()
        msg.data = 0.0
        for pub in self.pubs:
            pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MoverHaciaDelante()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_all()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
