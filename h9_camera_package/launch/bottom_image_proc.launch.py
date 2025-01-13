from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    composable_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::DebayerNode',
            name='debayer_node',
            namespace='bottom',
            parameters=[{'camera_namespace': 'bottom',}],
            remappings=[('/image_raw', '/bottom/image_raw'),
                        
                        ]
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_mono_node',
            namespace='bottom',
            parameters=[{'camera_namespace': 'bottom',}],
            # Remap subscribers and publishers
            remappings=[
                ('image', '/bottom/image_mono'),
                ('camera_info', '/bottom/camera_info'),
                ('image_rect', 'image_rect')
            ],
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_color_node',
            namespace='bottom',
            # Remap subscribers and publishers
            remappings=[
                ('image', '/bottom/image_color'),
                ('image_rect', 'image_rect_color')
            ],
        )
    ]

    arg_container = DeclareLaunchArgument(
        name='container', default_value='bottom_image_proc_container',
        description=(
            'Name of an existing node container to load launched nodes into. '
            'If unset, a new container will be created.'
        )
    )

    # If an existing container is not provided, start a container and load nodes into it
    image_processing_container = ComposableNodeContainer(
        condition=LaunchConfigurationEquals('container', 'bottom_image_proc_container'),
        name='image_proc_container',
        namespace='bottom',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen'
    )

    # If an existing container name is provided, load composable nodes into it
    # This will block until a container with the provided name is available and nodes are loaded
    load_composable_nodes = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals('container', 'bottom_image_proc_container'),
        composable_node_descriptions=composable_nodes,
        target_container=LaunchConfiguration('container'),
    )

    return LaunchDescription([
        arg_container,
        image_processing_container,
        load_composable_nodes,
    ])