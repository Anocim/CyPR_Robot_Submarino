import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64
import math

class PositionSetpointNode(Node):
    def __init__(self):
        super().__init__('position_setpoint_bridge')
        
        # --- SUSCRIPTORES ---
        # Escuchamos la Pose completa (X, Y, Z, R, P, Y)
        self.subscription = self.create_subscription(
            Pose,
            '/sub_target_pose',
            self.pose_callback,
            10)
        
        # --- PUBLICADORES ---
        # 1. Ejes de Posición (X, Y, Z)
        self.pub_x = self.create_publisher(Float64, '/target_x', 10)
        self.pub_y = self.create_publisher(Float64, '/target_y', 10)
        self.pub_depth = self.create_publisher(Float64, '/target_depth', 10) # El eje Z
        
        # 2. Ejes de Orientación (Roll, Pitch, Yaw)
        self.pub_orientation = self.create_publisher(Twist, '/target_orientation_euler', 10)

        self.get_logger().info("Nodo de Referencia 6-DOF Activo.")
        self.get_logger().info("Publicando en /target_x, /target_y, /target_depth y orientación.")

    def quaternion_to_euler(self, q):
        """Convierte quaternions a ángulos Euler (Roll, Pitch, Yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def pose_callback(self, msg):
        # --- MANEJO DE POSICIÓN (X, Y, Z) ---
        
        # Publicar X
        x_msg = Float64()
        x_msg.data = msg.position.x
        self.pub_x.publish(x_msg)

        # Publicar Y
        y_msg = Float64()
        y_msg.data = msg.position.y
        self.pub_y.publish(y_msg)

        # Publicar Z (Profundidad)
        z_msg = Float64()
        z_msg.data = msg.position.z
        self.pub_depth.publish(z_msg)

        # --- MANEJO DE ORIENTACIÓN (R, P, Y) ---
        r, p, y = self.quaternion_to_euler(msg.orientation)

        orientation_msg = Twist()
        orientation_msg.angular.x = r
        orientation_msg.angular.y = p
        orientation_msg.angular.z = y
        self.pub_orientation.publish(orientation_msg)

        self.get_logger().info(
            f"Nueva Ref -> X:{msg.position.x:.2f} Y:{msg.position.y:.2f} Z:{msg.position.z:.2f} | "
            f"Yaw:{math.degrees(y):.1f}°"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PositionSetpointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
