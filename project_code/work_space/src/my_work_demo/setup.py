from setuptools import find_packages, setup

package_name = 'my_work_demo'

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
    maintainer='qian',
    maintainer_email='16848811+qingfeng-dream@user.noreply.gitee.com',
    description='TODO: Package description',
    license='BSD',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'first = my_work_demo.first:main',
            'first_00 = my_work_demo.first_00:main',
            'calculate_secondary_waypoints = my_work_demo.calculate_secondary_waypoints:main'
        ],
    },
)

