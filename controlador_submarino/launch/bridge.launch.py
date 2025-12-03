from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Definimos la configuración de QoS para el puente
    # (Esto soluciona el error "incompatible QoS")
    qos_config = 'best_effort' 

    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='puente_motores',
            output='screen',
            parameters=[{'config_file': ''}], # No usamos fichero YAML, pasamos argumentos
            arguments=[
                # Motores (QoS configurado con @.../Float64@... )
                # NOTA: Gazebo Harmonic a veces necesita ayuda extra con el QoS.
                # Si esto falla, usaremos un fichero YAML.
                '/model/orca4/joint/thruster1_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/orca4/joint/thruster2_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/orca4/joint/thruster3_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/orca4/joint/thruster4_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/orca4/joint/thruster5_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/orca4/joint/thruster6_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                
                # Sensores
                '/world/sand/model/orca4/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                '/model/orca4/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'
            ]
        )
    ])
