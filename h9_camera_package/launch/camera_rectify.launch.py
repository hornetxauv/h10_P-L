from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Launch file for pool test.
# Inits 3 camera driver nodes, 1 gate detection node, and 1 pool lines detection node.

camera_init_delay = 5.0

camera_parameters = [
    {"width": 640},
    {"height": 480},
    {"codec": "unknown"},
    {"loop": 0},
    {"latency": 2000},
    {"framerate": 15.0},
]

# For v4l2_camera package, to reduce load on driver.
# camera_parameters = [
#     {"white_balance_temperature_auto": False},
#     {"white_balance_temperature": 5500},
#     {"brightness": 20},
#     {"output_encoding": "yuv422_yuy2"},
#     {"exposure_auto": 1},
#     {"backlight_compensation": 0},
#     {"time_per_frame": "[1,20]"},
# ]


def generate_launch_description():
    # Camera driver nodes
    leftcam = Node(
        package="ros_deep_learning",
        executable="video_source",
        name="left",
        namespace="left",
        output="screen",
        parameters=[
            {"resource": "v4l2:///dev/video0"},
            *camera_parameters,
        ],
    )

    rightcam = Node(
        package="ros_deep_learning",
        executable="video_source",
        name="right",
        namespace="right",
        output="screen",
        parameters=[
            {"resource": "v4l2:///dev/video2"},
            *camera_parameters,
        ],
    )

    bottomcam = Node(
        package="ros_deep_learning",
        executable="video_source",
        name="bottom",
        namespace="bottom",
        output="screen",
        parameters=[
            {"resource": "v4l2:///dev/video4"},
            *camera_parameters,
        ],
    )

    # Rectify Node
    rectify_node = Node(
        package="camera",
        executable="calibration",
        output="screen",
    )

    return LaunchDescription(
        [
            TimerAction(period=camera_init_delay, actions=[leftcam]),
            TimerAction(period=camera_init_delay * 2, actions=[rightcam]),
            TimerAction(period=camera_init_delay * 3, actions=[bottomcam]),
            rectify_node,
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
