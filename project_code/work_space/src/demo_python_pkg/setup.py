from setuptools import find_packages, setup

package_name = 'demo_python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='proxima',
    maintainer_email='proxima@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'python_node = demo_python_pkg.python_node:main',
            'mission_tist = demo_python_pkg.mission_tist:main',
            'offboard = demo_python_pkg.offboard:main',
            'yolo_to_point = demo_python_pkg.yolo_to_point:main',
        ],
    },
)
