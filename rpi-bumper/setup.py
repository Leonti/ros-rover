from setuptools import setup

package_name = 'rpi_bumper'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=[
        'rpi_bumper'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'RPi.GPIO'],
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
            'rpi_bumper = rpi_bumper:main'
        ],
    },
)
