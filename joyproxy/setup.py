from setuptools import setup

setup(
    name='joyproxy',
    version='0.0.0',
    packages=[],
    py_modules=['joyproxy_py'],
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
    description='Package containing a proxy from joystick commands to ROS_STPSIN commands',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'joyproxy_py = joyproxy_py:main'
        ],
    },
)
