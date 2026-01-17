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
    simulacion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_orca, 'launch', 'sim_launch.py')
        )
    )

    # 3. BRIDGE
    puente = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_propio, 'launch', 'bridge.launch.py')
        )
    )

    # 4. NODO CONTROLADOR
    controlador = Node(
        package='controlador_submarino',
        executable='control_sdre_perturbaciones', #<--- Cargar el controlador elegido
        name='controlador',
        output='screen'
    )

    # 5. NODO REFERENCIA (
    referencia = Node(
        package='controlador_submarino',
        executable='control_punto_punto', #<--- Cargar la referencia elegida
        name='referencia',
        output='screen'
    )

    return LaunchDescription([
        simulacion,
        puente,
        controlador,
        referencia
    ])
