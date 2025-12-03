import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class MapeoMotores(Node):
    def __init__(self):
        super().__init__('mapeo_motores')
        
        # Configuración QoS (Modo "Radio")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

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
            self.pubs.append(self.create_publisher(Float64, name, qos_profile))

        self.get_logger().info("--- INICIANDO MAPEO SUAVE ---")
        self.get_logger().info("Fuerza: 40N (Realista). Publicando a 20Hz.")
        
        # VARIABLES DE ESTADO
        self.motor_actual = 0
        self.fase = 0 # 0=Encender, 1=Apagar
        
        # DOS TEMPORIZADORES:
        # 1. El "Director": Cambia de motor cada 4 segundos
        self.timer_logica = self.create_timer(4.0, self.cambiar_motor)
        
        # 2. El "Obrero": Manda la fuerza constantemente (20 veces por segundo)
        self.timer_bucle = self.create_timer(0.05, self.enviar_fuerza)

    def cambiar_motor(self):
        """Este método solo decide QUÉ motor toca, no envía la fuerza."""
        if self.motor_actual >= 6:
            self.get_logger().info("--- FIN ---")
            self.stop_all() # Parada de seguridad
            return

        if self.fase == 0:
            # Toca ENCENDER
            self.get_logger().info(f"\n>>> PROBANDO MOTOR {self.motor_actual + 1} <<<")
            self.fase = 1
        else:
            # Toca APAGAR (Descanso)
            self.get_logger().info(f"Stop Motor {self.motor_actual + 1}")
            self.motor_actual += 1
            self.fase = 0

    def enviar_fuerza(self):
        """Este método se ejecuta muy rápido (20Hz) para mantener el motor vivo."""
        msg = Float64()
        
        # Si ya hemos terminado, no hacemos nada (o mandamos 0)
        if self.motor_actual >= 6:
            msg.data = 0.0
            return

        if self.fase == 1:
            # Estamos en fase de ENCENDIDO: Mandamos fuerza constante
            msg.data = 200.0  # <--- FUERZA REALISTA (No 500)
            self.pubs[self.motor_actual].publish(msg)
        else:
            # Estamos en fase de APAGADO: Mandamos ceros a todos
            self.stop_all()

    def stop_all(self):
        msg = Float64()
        msg.data = 0.0
        for pub in self.pubs:
            pub.publish(msg)

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
