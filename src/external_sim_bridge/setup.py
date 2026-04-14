from setuptools import find_packages, setup


package_name = 'external_sim_bridge'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/doc', ['doc/design_note.md']),
        (f'share/{package_name}/launch', ['launch/bridge_sim.launch.py']),
        (
            f'share/{package_name}/test',
            [
                'test/test_fake_backend.py',
                'test/test_bridge_node.py',
                'test/test_bridge_reference_launch.py',
            ],
        ),
        (
            f'share/{package_name}/test/data',
            ['test/data/dlqr_reference_trajectory.csv'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dylan Long',
    maintainer_email='dylan.long@ufl.edu',
    description='Generic ROS 2 bridge package for external simulator validation.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = external_sim_bridge.main:main',
        ],
    },
)
