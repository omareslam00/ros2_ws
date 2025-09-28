from setuptools import find_packages, setup

package_name = 'turtle_shapes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
   data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/turtle_shapes']),
    ('share/turtle_shapes', ['package.xml']),
    ('share/turtle_shapes/launch', ['launch/turtle_shapes.launch.py']), 
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omar',
    maintainer_email='omar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'shape_node = turtle_shapes.shape_node:main',
        'turtle_commander = turtle_shapes.turtle_commander:main',
        ],
    },
)
