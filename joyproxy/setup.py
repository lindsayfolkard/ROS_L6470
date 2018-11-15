from setuptools import setup

package_name='joyproxy'

setup(
    name=package_name,
    version='0.5.1',
    packages=[],
    py_modules=['joyproxy_py'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
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
    description='A proxy from joy commands to ROS_STPSIN commands',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joyproxy_py = joyproxy_py:main'
        ],
    },
)
