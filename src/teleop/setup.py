from setuptools import setup

package_name = 'teleop'

setup(
    name=teleop,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'teleop.motor_command_reader'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dylan Stone',
    maintainer_email='vha3kn@virginia.edu',
    description='Teleop package for training bot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motor_command_reader = teleop.motor_command_reader:main',
        ],
    },
)
