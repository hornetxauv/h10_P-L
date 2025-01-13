from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# This launch file launches the camera driver nodes for calibration, 
# hence no path to yaml file is specified

# video port for bottom camera is /dev/video0
# video port for left camera is /dev/video4
# video port for right camera is /dev/video2

def generate_launch_description():
    # Declare the paths to left and right camera calibration files
    bottom_camera_calibration_arg = DeclareLaunchArgument(
        'bottom_camera_calibration', 
        default_value='',
        description='Path to bottom camera calibration YAML file'
    )

    # camera driver nodes
    bottomcam = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="bottom",
        namespace="bottom",
        parameters=[
            {"video_device": "/dev/video0"},
            {"framerate": "30"},
            {"camera_info_url": LaunchConfiguration('bottom_camera_calibration')}
        ]
    )

    # Add your nodes to the launch description
    return LaunchDescription([
        bottom_camera_calibration_arg,
        bottomcam,
    ])

if __name__ == '__main__':
    generate_launch_description()
