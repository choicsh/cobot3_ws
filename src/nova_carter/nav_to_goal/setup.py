from setuptools import find_packages, setup

package_name = 'nav_to_goal'

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
    maintainer='rokey',
    maintainer_email='csh980625@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_test = nav_to_goal.move_test:main',
            'move_test_ljs = nav_to_goal.move_test_ljs:main',
            'move_test_followpath = nav_to_goal.move_test_followpath:main',
            'nav_through_pose = nav_to_goal.nav_through_pose:main',
            'image_saver_node = nav_to_goal.image_saver_node:main',
            'ground_truth_localization = nav_to_goal.ground_truth_localization:main',
            'nav_to_multi_pose = nav_to_goal.nav_to_multi_pose:main',
        ],
    },
)
