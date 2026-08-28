from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_pointcloud_corrector = LaunchConfiguration("use_pointcloud_corrector")
    use_ground_segmentor = LaunchConfiguration("use_ground_segmentor")
    use_occupancy_mapping = LaunchConfiguration("use_occupancy_mapping")
    use_field_map_builder = LaunchConfiguration("use_field_map_builder")
    use_global_planner = LaunchConfiguration("use_global_planner")
    use_path_optimizer = LaunchConfiguration("use_path_optimizer")
    use_local_planner = LaunchConfiguration("use_local_planner")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    autostart = LaunchConfiguration("autostart")
    use_sim_time = LaunchConfiguration("use_sim_time")

    default_params_file = PathJoinSubstitution([
        FindPackageShare("trailblazer_bringup"),
        "config",
        "bringup.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Full path to TrailBlazer bringup parameter file.",
        ),

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock if true.",
        ),

        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically configure and activate lifecycle nodes.",
        ),

        DeclareLaunchArgument(
            "use_lifecycle_manager",
            default_value="true",
            description="Start lifecycle manager.",
        ),

        DeclareLaunchArgument(
            "use_pointcloud_corrector",
            default_value="true",
            description="Start pointcloud corrector server.",
        ),

        DeclareLaunchArgument(
            "use_ground_segmentor",
            default_value="true",
            description="Start ground segmentor server.",
        ),

        DeclareLaunchArgument(
            "use_occupancy_mapping",
            default_value="true",
            description="Start occupancy mapping server.",
        ),

        DeclareLaunchArgument(
            "use_field_map_builder",
            default_value="true",
            description="Start field mapping server.",
        ),

        DeclareLaunchArgument(
            "use_global_planner",
            default_value="true",
            description="Start global planner server.",
        ),

        DeclareLaunchArgument(
            "use_path_optimizer",
            default_value="true",
            description="Start path optimizer server.",
        ),

        DeclareLaunchArgument(
            "use_local_planner",
            default_value="true",
            description="Start local planner server.",
        ),

        Node(
            package="pointcloud_corrector",
            executable="pointcloud_corrector_server",
            name="pointcloud_corrector_server",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": use_sim_time},
            ],
            condition=IfCondition(use_pointcloud_corrector),
        ),

        Node(
            package="ground_segmentor",
            executable="ground_segmentor_server",
            name="ground_segmentor_server",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": use_sim_time},
            ],
            condition=IfCondition(use_ground_segmentor),
        ),

        Node(
            package="occupancy_mapping",
            executable="occupancy_mapping_server",
            name="occupancy_mapping_server",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": use_sim_time,
                },
            ],
            condition=IfCondition(use_occupancy_mapping),
        ),

        Node(
            package="field_map_builder",
            executable="field_map_builder_server",
            name="field_map_builder_server",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": use_sim_time,
                },
            ],
            condition=IfCondition(use_field_map_builder),
        ),

        Node(
            package="global_planner",
            executable="global_planner_server",
            name="global_planner_server",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": use_sim_time,
                },
            ],
            condition=IfCondition(use_global_planner),
        ),

        Node(
            package="path_optimizer",
            executable="path_optimizer_server",
            name="path_optimizer_server",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": use_sim_time,
                },
            ],
            condition=IfCondition(use_path_optimizer),
        ),

        Node(
            package="local_planner",
            executable="local_planner_server",
            name="local_planner_server",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": use_sim_time,
                },
            ],
            condition=IfCondition(use_local_planner),
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_trailblazer",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "bond_timeout": 0.0,
                },
            ],
            condition=IfCondition(use_lifecycle_manager),
        ),
    ])