from glob import glob
from setuptools import setup

setup(
    name='a2_demo', version='1.0.0', packages=['a2_demo'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/a2_demo']),
        ('share/a2_demo', ['package.xml']),
        ('share/a2_demo/launch', glob('launch/*.launch.py')),
        ('share/a2_demo/urdf', glob('urdf/*.urdf')),
        ('share/a2_demo/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='A2 Course Team', maintainer_email='team@example.com',
    description='A2 machine tending teaching demo', license='MIT',
    entry_points={'console_scripts': [
        'arm_sim = a2_demo.arm_node:main',
        'machine_sim = a2_demo.machine_node:main',
        'task_manager = a2_demo.task_node:main',
        'scene_view = a2_demo.scene_node:main',
        'acceptance = a2_demo.acceptance:main',
    ]},
)
