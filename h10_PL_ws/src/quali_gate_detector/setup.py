from setuptools import setup

package_name = 'quali_gate_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hornetx',
    maintainer_email='hornetxauv2425@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector = quali_gate_detector.detector:main',
            'old_detector = quali_gate_detector.old_detector:main',
            'detector_listener = quali_gate_detector.detector_listener:main'
        ],
    },
)
