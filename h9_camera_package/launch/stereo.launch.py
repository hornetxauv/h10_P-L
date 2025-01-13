from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# This launch file launches the camera driver nodes after calibration, hence the path to yaml file is specified
# Launch this file at ~/camera_ws/

def generate_launch_description():
    # Declare the paths to left and right camera calibration files
    left_camera_calibration_arg = DeclareLaunchArgument(
        'left_camera_calibration', 
        # relative path to calibration file, with respect to ROS2 workspace
        default_value='file:///home/bb/poolTest_ws/src/camera_ws/camera/calibration/calibrationdata/left.yaml',
        description='Path to left camera calibration YAML file'
    )
    
    right_camera_calibration_arg = DeclareLaunchArgument(
        'right_camera_calibration', 
        # relative path to calibration file, with respect to ROS2 workspace
        default_value='file:///home/bb/poolTest_ws/src/camera_ws/camera/calibration/calibrationdata/right.yaml',
        description='Path to right camera calibration YAML file'
    )  

    bottom_camera_calibration_arg = DeclareLaunchArgument(
        'bottom_camera_calibration', 
        # relative path to calibration file, with respect to ROS2 workspace
        default_value='file:///home/bb/poolTest_ws/src/camera_ws/camera/calibration/calibrationdata/bottom.yaml',
        description='Path to bottom camera calibration YAML file'
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
    bottomcam = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="bottom",
        namespace="bottom",
        parameters=[
            {"video_device": "/dev/video0"},
            {"camera_info_url": LaunchConfiguration('bottom_camera_calibration')}
        ]
    )

    # Add your nodes to the launch description
    return LaunchDescription([
        left_camera_calibration_arg, 
        right_camera_calibration_arg, 
        bottom_camera_calibration_arg, 
        leftcam,
        rightcam,
        bottomcam,
    ])

if __name__ == '__main__':
    generate_launch_description()
