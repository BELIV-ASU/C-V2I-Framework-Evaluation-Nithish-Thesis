from setuptools import find_packages, setup

package_name = 'sim_c_v2i_evaluation'

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
        'console_scripts': ['ground_truth_objects_parser = sim_c_v2i_evaluation.ground_truth_objects_parser:main',
                            'carom_before_delay_compensation_parser = sim_c_v2i_evaluation.carom_before_delay_compensation_parser:main',
                            'carom_after_delay_compensation_parser = sim_c_v2i_evaluation.carom_after_delay_compensation_parser:main',
                            'merged_objects_parser = sim_c_v2i_evaluation.merged_objects_parser:main',
                            'objects_frame_transformation_baselink_to_map = sim_c_v2i_evaluation.objects_frame_transformation_baselink_to_map:main',
                            'onboard_objects_parser = sim_c_v2i_evaluation.onboard_objects_parser:main'
        ],
    },
)
