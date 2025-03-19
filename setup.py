from setuptools import find_packages, setup

package_name = 'infrastructure_objects_frame_transformation'

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
    maintainer='nithish',
    maintainer_email='nithish@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['objects_frame_transformation = infrastructure_objects_frame_transformation.objects_frame_transformation:main',
        'objects_frame_transformation_version2 = infrastructure_objects_frame_transformation.objects_frame_transformation_version2:main',
        'objects_frame_transformation_version3 = infrastructure_objects_frame_transformation.objects_frame_transformation_version3:main',
        'base_to_camera_frame_tf_broadcaster = infrastructure_objects_frame_transformation.base_to_camera_frame_tf_broadcaster:main'
        ],
    },
)
