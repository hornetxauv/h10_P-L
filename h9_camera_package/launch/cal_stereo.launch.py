from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# This launch file launches the camera driver nodes for STEREO calibration, 
# hence no path to yaml file is specified

# video port for left camera is /dev/video4
# video port for right camera is /dev/video2

def generate_launch_description():
    # Declare the paths to left and right camera calibration files
    left_camera_calibration_arg = DeclareLaunchArgument(
        'left_camera_calibration', 
        default_value='',
        description='Path to left camera calibration YAML file'
    )
    
    right_camera_calibration_arg = DeclareLaunchArgument(
        'right_camera_calibration', 
        default_value='',
        description='Path to right camera calibration YAML file'
    )

    # camera driver nodes
    leftcam = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="left",
        namespace="left",
        parameters=[
            {"video_device": "/dev/video4"},
            {"camera_info_url": LaunchConfiguration('left_camera_calibration')}
        ]
    )
    rightcam = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="right",
        namespace="right",
        parameters=[
            {"video_device": "/dev/video2"},
            {"camera_info_url": LaunchConfiguration('right_camera_calibration')}
        ]
    )

    # Add your nodes to the launch description
    return LaunchDescription([
        left_camera_calibration_arg, 
        right_camera_calibration_arg, 
        leftcam,
        rightcam,
    ])

if __name__ == '__main__':
    generate_launch_description()