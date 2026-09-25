from setuptools import find_packages, setup

package_name = 'hospital_system'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='barlide',
    maintainer_email='alqp201@gmail.com',
    description='Hospital specimen transport system: robot agent, fleet manager, DB',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_agent = hospital_system.robot_agent:main',
            'db_worker = hospital_system.db_worker:main',
        ],
    },
)
