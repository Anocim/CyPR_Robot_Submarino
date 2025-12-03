import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class MapeoMotores(Node):
    def __init__(self):
        super().__init__('mapeo_motores')
        
        # Configuración de QoS para que Gazebo nos haga caso (Best Effort)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Lista de topics (Asegúrate de que coinciden con tu gz topic -l)
        self.topic_names = [
            '/model/orca4/joint/thruster1_joint/cmd_thrust',
            '/model/orca4/joint/thruster2_joint/cmd_thrust',
            '/model/orca4/joint/thruster3_joint/cmd_thrust',
            '/model/orca4/joint/thruster4_joint/cmd_thrust',
            '/model/orca4/joint/thruster5_joint/cmd_thrust',
            '/model/orca4/joint/thruster6_joint/cmd_thrust'
        ]

        self.pubs = []
        for name in self.topic_names:
            # Usamos el perfil QoS aquí
            self.pubs.append(self.create_publisher(Float64, name, qos_profile))

        self.get_logger().info("--- INICIANDO MAPEO (QoS: Best Effort) ---")
        self.get_logger().info("¡Mira el robot! Enviando 500N.")
        
        self.timer = self.create_timer(3.0, self.prueba_secuencial)
        self.motor_actual = 0
        self.fase = 0 # 0=Encender, 1=Apagar

    def stop_all(self):
        msg = Float64()
        msg.data = 0.0
        for pub in self.pubs:
            pub.publish(msg)

    def prueba_secuencial(self):
        if self.motor_actual >= 6:
            self.get_logger().info("--- FIN ---")
            self.stop_all()
            return

        msg = Float64()
        
        if self.fase == 0:
            # ENCENDER (Fuerza alta para que se note)
            msg.data = 500.0 
            self.pubs[self.motor_actual].publish(msg)
            self.get_logger().info(f">>> MOTOR {self.motor_actual + 1} ENCENDIDO <<<")
            self.fase = 1
        else:
            # APAGAR
            self.stop_all()
            self.get_logger().info(f"Stop Motor {self.motor_actual + 1}")
            self.motor_actual += 1
            self.fase = 0

def main(args=None):
    rclpy.init(args=args)
    node = MapeoMotores()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_all()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
