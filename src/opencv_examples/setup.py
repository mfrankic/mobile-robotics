from setuptools import setup

package_name = 'opencv_examples'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Diego Susanj',
    maintainer_email='dsusanj@riteh.hr',
    description='Package containing multiple ROS2 OpenCV examples.',
    license='GNU GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_player = opencv_examples.camera_player:main',
            'video_player = opencv_examples.video_player:main',
            'color_thresholding = opencv_examples.color_thresholding:main',
            'edge_detection = opencv_examples.edge_detection:main',
            'feature_matching = opencv_examples.feature_matching:main',
            'people_detection = opencv_examples.people_detection:main',
            'face_detection = opencv_examples.face_detection:main',
            'object_recognition = opencv_examples.object_recognition:main'
        ],
    },
)
