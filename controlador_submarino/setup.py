import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'controlador_submarino'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carmenmunncar',
    maintainer_email='carmenmc2705@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cerebro = controlador_submarino.controlador:main',
            'mapeo = controlador_submarino.mapeo:main',
            'angulos=controlador_submarino.angulos:main',
            'controlador_curro=controlador_submarino.controlador_curro:main',
            'controlador_edu=controlador_submarino.controlador2:main',
            'mover=controlador_submarino.mover:main',
            'controlador_dinamica=controlador_submarino.control_dinamica:main',
            'controlador_PID=controlador_submarino.controlador_PID:main',
            'control_din_PID=controlador_submarino.control_din_PID:main',
            'control_din_PID_odom=controlador_submarino.control_din_PID_odom:main',
            'control_din_PID_odom_cambio=controlador_submarino.control_din_PID_odom_cambio:main',
            'cambio_ref=controlador_submarino.cambio_ref:main',
            'control_sdre=controlador_submarino.control_sdre:main',
            'cambio_pos=controlador_submarino.cambio_pos:main',
        ],
    },
)
