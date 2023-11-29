from setuptools import find_packages, setup

package_name = 'pubsub_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sis',
    maintainer_email='sis@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ #start node via console script  
            'pub = pubsub_py.pub:main',
            'sub = pubsub_py.sub:main',
        ],
    },
)
