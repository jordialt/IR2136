from setuptools import setup

package_name = 'my_drone_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jordi Altarejos',
    maintainer_email='al416445@uji.es',
    description='Paquete para controlar el dron en ROS 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'battery_gps_node = my_drone_package.battery_gps_node:main',
            'mission_control_node = my_drone_package.mission_control_node:main',
        ],
    },
    package_data={
        # Esto asegura que el archivo 'package.xml' se incluya en la instalación
        '': ['package.xml'],
    },
)


