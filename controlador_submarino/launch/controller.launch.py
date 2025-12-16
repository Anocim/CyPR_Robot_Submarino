import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # 1. LOCALIZAR LOS PAQUETES
    pkg_orca = get_package_share_directory('orca_bringup')
    pkg_propio = get_package_share_directory('controlador_submarino')

    # 2. INCLUIR LA SIMULACIÓN (Mundo y Robot)
    # Nota: He corregido 'sim_laun.py' a 'sim_launch.py' que suele ser el nombre correcto.
    # Si tu archivo se llama realmente 'sim_laun.py', cámbialo abajo.
    simulacion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_orca, 'launch', 'sim_launch.py')
        )
    )

    # 3. INCLUIR EL PUENTE (Bridge)
    puente = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_propio, 'launch', 'bridge.launch.py')
        )
    )

    # 4. NODO CONTROLADOR (Tu PID)
    # "executable" debe coincidir con el nombre que pusiste en setup.py (entry_points)
    controlador = Node(
        package='controlador_submarino',
        executable='control_din_PID_odom_cambio', # <--- Asegúrate que este nombre está en setup.py
        name='controlador_pid_dinamico',
        output='screen'
    )

    # 5. NODO REFERENCIA (Tu generador de cambios)
    referencia = Node(
        package='controlador_submarino',
        executable='cambio_ref', # <--- Asegúrate que este nombre está en setup.py
        name='referencia_cambio',
        output='screen'
    )

    return LaunchDescription([
        simulacion,
        puente,
        controlador,
        referencia
    ])
