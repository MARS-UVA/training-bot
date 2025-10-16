from setuptools import setup, find_packages

package_name = 'teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src'),  # or packages=[package_name] if your code is flat in src/teleop
    package_dir={'': 'src'},               # assuming code is in src/teleop now
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
