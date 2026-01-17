import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench

class CorrienteConstante(Node):
    def __init__(self):
        super().__init__('generador_corriente_marina')
        
        # Publicamos en el tópico de fuerza externa del robot
        self.publisher_ = self.create_publisher(
            Wrench, 
            '/model/orca4/link/base_link/wrench', 
            10
        )
        
        # Simulamos una corriente de 0.5 m/s
        # Fuerza = Coeficiente_Arrastre (33.8) * Velocidad^2 (0.5^2) = 8.45 N
        self.fuerza_corriente = 5000 
        
        # Timer para enviar la fuerza constantemente (20 veces por segundo)
        self.timer = self.create_timer(0.05, self.publicar_fuerza)
        self.get_logger().info(f'--- CORRIENTE MARINA ACTIVA: {self.fuerza_corriente} N en Eje X ---')

    def publicar_fuerza(self):
        msg = Wrench()
        # Aplicamos la fuerza en X (simulando corriente de proa/popa)
        msg.force.x = 0.0
        msg.force.y = self.fuerza_corriente
        msg.force.z = 0.0
        msg.torque.x = 0.0
        msg.torque.y = 0.0
        msg.torque.z = 0.0
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CorrienteConstante()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()