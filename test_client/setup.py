#from ament_python.data_files import get_data_files
from ament_python.script_dir import install_scripts_to_libexec
from setuptools import setup

package_name = 'l6470_test_client'
#data_files = get_data_files(package_name)
install_scripts_to_libexec(package_name)

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=['l6470_client','l6470_publisher'],
    #data_files=data_files,
    install_requires=['setuptools'],
    author='Lindsay Folkard',
    author_email='lindsayfolkard@gmail.com',
    maintainer='Lindsay Folkard',
    maintainer_email='lindsayfolkard@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Package containing examples of how to use the rclpy API.',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'l6470_client = l6470_client:main',
            'l6470_publiser = l6470_publisher:main'
        ],
    },
)
