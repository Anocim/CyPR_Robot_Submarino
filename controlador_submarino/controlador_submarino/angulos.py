import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
from sensor_msgs.msg import FluidPressure
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan

class ControladorSubmarino(Node):
    def __init__(self):
        super().__init__('controlador_submarino')

        qos_profile = QoSProfile(
           reliability=ReliabilityPolicy.RELIABLE,
           durability=DurabilityPolicy.VOLATILE,
           history=HistoryPolicy.KEEP_LAST,
           depth=10
        )

        # Variables para guardar los ángulos (inicializadas a 0)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Suscribirse a la Odometría
        self.sub_odom = self.create_subscription(
            Odometry,
            '/model/orca4/odometry', 
            self.listener_callback,
            qos_profile)
        
        self.sub_altimeter = self.create_subscription(
    	    LaserScan,
    	    '/world/sand/model/orca4/link/altimeter_link/sensor/altimeter_sensor/scan',
    	    self.altimeter_callback,
    	    qos_profile)
            
        self.get_logger().info("--- CONTROLADOR INICIADO ---")
        self.get_logger().info("Esperando datos de odometría para mostrar ángulos...")

    def listener_callback(self, msg):
    
        profundidad_real = msg.pose.pose.position.z
        # 1. Extraer el cuaternión del mensaje
        q = msg.pose.pose.orientation
        
        # 2. Convertirlo a lista [x, y, z, w]
        quaternion_list = [q.x, q.y, q.z, q.w]

        # 3. Magia matemática: Cuaternión -> Euler
        (roll, pitch, yaw) = euler_from_quaternion(quaternion_list)

        # 4. Guardarlo en las variables de clase
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        self.get_logger().info(f'Profund: {profundidad_real:.2f} | Roll: {roll_deg:.2f}° | Pitch: {pitch_deg:.2f}° | Yaw: {yaw_deg:.2f}°')
    	
    def altimeter_callback(self, msg):
    	distancia_fondo = msg.ranges[0]
    
    	# Filtrar infinitos (si no ve fondo)
    	if distancia_fondo == float('inf'):
            distancia_fondo = 50.0 # O el máximo

    	self.get_logger().info(f"Distancia al fondo: {distancia_fondo:.2f} m")

def main(args=None):
    rclpy.init(args=args)
    node = ControladorSubmarino()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
