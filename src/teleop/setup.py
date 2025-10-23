from setuptools import setup

package_name = 'teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # this assumes you have a teleop/ folder with __init__.py inside
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ackage',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This registers the Python node as a console executable
            'motor_command_reader = scripts.motor_command_reader:main',
        ],
    },
)
