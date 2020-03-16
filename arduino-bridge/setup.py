from setuptools import setup

package_name = 'arduino_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=[
        'arduino_bridge',
        'transformations_new'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: Apache 2.0',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_bridge = arduino_bridge:main'
        ],
    },
)
