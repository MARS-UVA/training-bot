from setuptools import find_packages, setup

package_name = 'webcam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy<2',
        'opencv-python'
    ],
    zip_safe=True,
    maintainer='Ivan Post',
    maintainer_email='ivan.post24@gmail.com',
    description='Read images in real time from a webcam using OpenCV.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'webcam_capture = webcam.node:main'
        ],
    },
)
