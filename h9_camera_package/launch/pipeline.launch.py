# from controls_core.params import CAM_FLAGS, CAM_PARAMS
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

calibration_data_dir = (
    "/home/bb/poolTest_ws/src/camera_ws/camera/calibration/calibrationdata/"
)


def evaluate_launch(context, *args, **kwargs):
    cam_name = LaunchConfiguration("cam_name").perform(context)
    cam_init_delay = float(LaunchConfiguration("cam_init_delay").perform(context))

    # # Camera driver nodes
    # cam_node = Node(
    #     package="ros_deep_learning",
    #     executable="video_source",
    #     namespace=cam_name,
    #     output="screen",
    #     parameters=[
    #         {"resource": f"v4l2://{CAM_FLAGS[cam_name]['dev']}"},
    #         *CAM_PARAMS,
    #     ],
    #     remappings=[("raw", "image")],
    # )

    # cam_info_node = Node(
    #     package="camera",
    #     executable="calibration",
    #     parameters=[
    #         {"calibration_data_path": f"{calibration_data_dir}{cam_name}.yaml"}
    #     ],
    #     output="screen",
    # )

    # # Rectify Node - changed to the new rectification
    # rectify_node = Node(
    #     package="camera",
    #     executable="stereo_rectify",
    #     # parameters=[{"calibration_data_path": f"{calibration_data_dir}left.yaml"}],
    #     output="screen",
    # )

    # compressed_node = Node(
    #     package="camera",
    #     executable="compressed",
    #     output="screen",
    # )

    # compressed_v2_node = Node(
    #     package="camera",
    #     executable="compressed_v2",
    #     output="screen",
    # )

    # # Launch lines.py
    # lines_node = Node(
    #     package="camera",
    #     executable="lines",
    #     output="screen",
    # )

    yolov8_visualizer_node = Node(
        namespace=f"{cam_name}/rect",
        package="camera",
        executable="isaac_ros_yolov8_visualizer",
        output="screen",
    )

    # launch_nodes = []
    # if CAM_FLAGS[cam_name]["raw"]:
    #     launch_nodes.append(TimerAction(period=cam_init_delay, actions=[cam_node]))
    #     launch_nodes.append(compressed_node)

    #     if CAM_FLAGS[cam_name]["calibrated"]:
    #         launch_nodes.append(cam_info_node)
    #         launch_nodes.append(compressed_v2_node)

    #     if CAM_FLAGS[cam_name]["yolo"]:
    #         launch_nodes.append(yolov8_visualizer_node)

    return [yolov8_visualizer_node]


def generate_launch_description():
    cam_name_launch_arg = DeclareLaunchArgument("cam_name", default_value="left")
    cam_init_delay_launch_arg = DeclareLaunchArgument(
        "cam_init_delay", default_value="5.0"
    )

    return LaunchDescription(
        [
            cam_name_launch_arg,
            cam_init_delay_launch_arg,
            OpaqueFunction(function=evaluate_launch),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
