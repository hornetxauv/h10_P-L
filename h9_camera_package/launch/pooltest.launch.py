from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare

# Launch file for pool test.
# Inits 3 camera driver nodes, 1 gate detection node, and 1 pool lines detection node.

# cam_init_delay = 5.0


calibration_data_dir = (
    "/home/bb/poolTest_ws/src/camera_ws/camera/calibration/calibrationdata/"
)


def generate_launch_description():
    pipeline_groups = []

    for i, cam_name in enumerate(["left", "right", "bottom"]):
        pipeline_groups.append(
            GroupAction(
                actions=[
                    # PushRosNamespace(cam_name),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [FindPackageShare("camera"), "/launch/pipeline.launch.py"]
                        ),
                        launch_arguments={
                            "cam_name": cam_name,
                            # "cam_init_delay": f"{cam_init_delay * (i + 1)}",
                        }.items(),
                    ),
                ]
            )
        )

    return LaunchDescription(pipeline_groups)


if __name__ == "__main__":
    generate_launch_description()
