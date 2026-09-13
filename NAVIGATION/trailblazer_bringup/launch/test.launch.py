import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import (
    ComposableNodeContainer,
    Node,
)
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


# The component_plugin value must exactly match the PLUGIN string used by
# rclcpp_components_register_node(...) in the corresponding package.
SERVER_SPECS = (
    {
        "package": "pointcloud_corrector",
        "executable": "pointcloud_corrector_server",
        "name": "pointcloud_corrector_server",
        "enable_arg": "use_pointcloud_corrector",
        "intra_process_arg": "use_pointcloud_corrector_intra_process_comms",        # 进程内通信，只有放入共享容器内有效
        "intra_process_default": "true",
        "composition_group_arg": "pointcloud_corrector_composition_group",
        "composition_group_default": "perception",
        "component_arg": "pointcloud_corrector_component",
        "component_plugin": "pointcloud_corrector::PointcloudCorrectorServer",
    },
    {
        "package": "ground_segmentor",
        "executable": "ground_segmentor_server",
        "name": "ground_segmentor_server",
        "enable_arg": "use_ground_segmentor",
        "intra_process_arg": "use_ground_segmentor_intra_process_comms",
        "intra_process_default": "true",
        "composition_group_arg": "ground_segmentor_composition_group",
        "composition_group_default": "perception",
        "component_arg": "ground_segmentor_component",
        "component_plugin": "ground_segmentor::GroundSegmentorServer",
    },
    {
        "package": "occupancy_mapping",
        "executable": "occupancy_mapping_server",
        "name": "occupancy_mapping_server",
        "enable_arg": "use_occupancy_mapping",
        "intra_process_arg": "use_occupancy_mapping_intra_process_comms",
        "intra_process_default": "false",
        "composition_group_arg": "occupancy_mapping_composition_group",
        "composition_group_default": "mapping",
        "component_arg": "occupancy_mapping_component",
        "component_plugin": "occupancy_mapping::OccupancyMappingServer",
    },
    {
        "package": "field_map_builder",
        "executable": "field_map_builder_server",
        "name": "field_map_builder_server",
        "enable_arg": "use_field_map_builder",
        "intra_process_arg": "use_field_map_builder_intra_process_comms",
        "intra_process_default": "false",
        "composition_group_arg": "field_map_builder_composition_group",
        "composition_group_default": "mapping",
        "component_arg": "field_map_builder_component",
        "component_plugin": "field_map_builder::FieldMapBuilderServer",
    },
    {
        "package": "global_planner",
        "executable": "global_planner_server",
        "name": "global_planner_server",
        "enable_arg": "use_global_planner",
        "intra_process_arg": "use_global_planner_intra_process_comms",
        "intra_process_default": "false",
        "composition_group_arg": "global_planner_composition_group",
        "composition_group_default": "planning",
        "component_arg": "global_planner_component",
        "component_plugin": "global_planner::GlobalPlannerServer",
    },
    {
        "package": "path_optimizer",
        "executable": "path_optimizer_server",
        "name": "path_optimizer_server",
        "enable_arg": "use_path_optimizer",
        "intra_process_arg": "use_path_optimizer_intra_process_comms",
        "intra_process_default": "false",
        "composition_group_arg": "path_optimizer_composition_group",
        "composition_group_default": "planning",
        "component_arg": "path_optimizer_component",
        "component_plugin": "path_optimizer::PathOptimizerServer",
    },
    {
        "package": "local_planner",
        "executable": "local_planner_server",
        "name": "local_planner_server",
        "enable_arg": "use_local_planner",
        "intra_process_arg": "use_local_planner_intra_process_comms",
        "intra_process_default": "false",
        "composition_group_arg": "local_planner_composition_group",
        "composition_group_default": "standalone",
        "component_arg": "local_planner_component",
        "component_plugin": "local_planner::LocalPlannerServer",
    },
)


_VALID_ROS_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def _is_enabled(context, launch_argument):
    return IfCondition(LaunchConfiguration(launch_argument)).evaluate(context)


def _launch_servers(context):
    """Create standalone processes and one component container per group.

    With use_composition=false every enabled server is standalone. With it set
    to true, a server whose composition_group is "standalone" is still a
    separate process; servers with the same non-empty group share one container.
    """
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_composition = _is_enabled(context, "use_composition")       # 设为true：标记为standalone的节点以独立进程运行，其余配置了相同分组名的节点，聚合到同一个共享组件容器里运行

    container_name_prefix = LaunchConfiguration("container_name").perform(context).strip()
    if not _VALID_ROS_NAME.fullmatch(container_name_prefix):
        raise RuntimeError(
            "container_name must be a valid ROS node-name prefix; got "
            f"{container_name_prefix!r}"
        )

    container_executable = LaunchConfiguration(
        "component_container_executable"
    ).perform(context).strip()
    supported_containers = {
        "component_container",
        "component_container_mt",
        "component_container_isolated",
    }
    if container_executable not in supported_containers:
        raise RuntimeError(
            "component_container_executable must be one of "
            f"{sorted(supported_containers)}; got {container_executable!r}"
        )

    try:
        container_thread_num = int(
            LaunchConfiguration("component_container_thread_num").perform(context)
        )
    except ValueError as exc:
        raise RuntimeError(
            "component_container_thread_num must be a non-negative integer"
        ) from exc
    if container_thread_num < 0:
        raise RuntimeError(
            "component_container_thread_num must be a non-negative integer"
        )

    isolated_uses_mte = _is_enabled(
        context, "isolated_use_multi_threaded_executor"
    )

    standalone_actions = []                 # 用来放独立进程节点的启动动作
    components_by_group = {}                # 一个字典，分组名->节点

    for spec in SERVER_SPECS:
        if not _is_enabled(context, spec["enable_arg"]):
            continue

        group = "standalone"
        if use_composition:
            group = LaunchConfiguration(
                spec["composition_group_arg"]
            ).perform(context).strip()

        if not group:
            raise RuntimeError(
                f"{spec['composition_group_arg']} must not be empty"
            )

        if group == "standalone":                   # 独立节点启动
            standalone_actions.append(
                Node(
                    package=spec["package"],
                    executable=spec["executable"],
                    name=spec["name"],
                    output="screen",
                    parameters=[
                        params_file,
                        {"use_sim_time": use_sim_time},
                    ],
                )
            )
            continue

        if not _VALID_ROS_NAME.fullmatch(group):            # 分组名字不合法
            raise RuntimeError(
                f"{spec['composition_group_arg']} must be 'standalone' or a "
                f"valid ROS name; got {group!r}"
            )

        components_by_group.setdefault(group, []).append(
            ComposableNode(
                package=spec["package"],
                plugin=LaunchConfiguration(spec["component_arg"]),
                name=spec["name"],
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time},
                ],
                extra_arguments=[{
                    "use_intra_process_comms": LaunchConfiguration(
                        spec["intra_process_arg"]
                    ),
                }],
            )
        )

    container_actions = []
    for group, components in components_by_group.items():
        # component_container_mt otherwise defaults to hardware_concurrency(),
        # which is often unnecessarily large for a small component group.
        container_parameters = []
        if container_executable == "component_container_mt":
            container_parameters.append({"thread_num": container_thread_num})

        container_arguments = []
        if (
            container_executable == "component_container_isolated"
            and isolated_uses_mte
        ):
            # This creates one dedicated MTE per component, not one shared MTE.
            container_arguments.append("--use_multi_threaded_executor")

        container_actions.append(
            ComposableNodeContainer(
                package="rclcpp_components",
                executable=container_executable,
                name=f"{container_name_prefix}_{group}",
                namespace="",
                output="screen",
                parameters=container_parameters,
                arguments=container_arguments,
                composable_node_descriptions=components,
            )
        )

    return [*container_actions, *standalone_actions]


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    autostart = LaunchConfiguration("autostart")
    use_sim_time = LaunchConfiguration("use_sim_time")

    default_params_file = PathJoinSubstitution([
        FindPackageShare("trailblazer_bringup"),
        "config",
        "bringup.yaml",
    ])

    declared_arguments = [
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
            "use_composition",
            default_value="true",
            description=(
                "Enable per-server composition_group routing. If false, every "
                "enabled server runs as a standalone process."
            ),
        ),
        DeclareLaunchArgument(
            "container_name",
            default_value="trailblazer_container",
            description=(
                "Container name prefix; each group becomes "
                "<prefix>_<composition_group>."
            ),
        ),
        # component_container：基础单线程容器，所有节点共用同一个单线程执行器，同一时间只能处理一个回调任务，不会出现线程竞争问题，适合低算力、逻辑简单的节点组
        # component_container_mt：多线程容器，所有节点共用同一个多线程执行器，支持并行处理多个回调任务，兼顾性能和易用性
        # component_container_isolated：隔离执行器容器，每个节点默认拥有独立的单线程执行器，节点之间的回调完全互不阻塞，避免单个节点的耗时任务拖慢整个容器内的其他节点
        DeclareLaunchArgument(
            "component_container_executable",
            default_value="component_container_mt",
            description=(
                "rclcpp_components container executable: component_container_mt, "
                "component_container_isolated, or component_container."
            ),
        ),
        # 仅对多线程容器生效，指定component_container_mt多线程容器内部的执行器工作线程数量
        # 设为0：自动读取当前设备的CPU核心数，创建和核心数相等的线程
        DeclareLaunchArgument(
            "component_container_thread_num",
            default_value="2",
            description=(
                "Threads per component_container_mt. Set 0 to use the CPU core "
                "count. Ignored by the other container executables."
            ),
        ),
        # 仅对隔离容器生效的全局配置项，指定component_container_isolated隔离容器内，每个节点的专属执行器类型
        # false：每个节点默认分配独立的单线程执行器，同一时间单个节点只能处理一个回调任务，线程调度开销极低
        # true：给容器内的每个组件单独分配专属的多线程执行器，单个节点内部就可以并行处理多个回调任务，适合高算力、单个节点内部回调任务很多的高性能部署场景
        DeclareLaunchArgument(
            "isolated_use_multi_threaded_executor",
            default_value="true",
            description=(
                "For component_container_isolated only: give each component its "
                "own MultiThreadedExecutor instead of its own SingleThreadedExecutor."
            ),
        ),
    ]

    for spec in SERVER_SPECS:
        declared_arguments.extend([
            DeclareLaunchArgument(
                spec["enable_arg"],
                default_value="true",
                description=f"Start {spec['name']}.",
            ),
            DeclareLaunchArgument(
                spec["component_arg"],
                default_value=spec["component_plugin"],
                description=f"Registered component plugin for {spec['name']}.",
            ),
            DeclareLaunchArgument(
                spec["intra_process_arg"],
                default_value=spec["intra_process_default"],
                description=(
                    "Enable rclcpp intra-process communication for "
                    f"{spec['name']}."
                ),
            ),
            DeclareLaunchArgument(
                spec["composition_group_arg"],
                default_value=spec["composition_group_default"],
                description=(
                    "When use_composition=true, use 'standalone' for a separate "
                    "process or assign a group name. Servers with the same group "
                    f"share one container. Applies to {spec['name']}."
                ),
            ),
        ])

    # It can also be composed if desired, but keeping the lifecycle manager in
    # its own process gives the manager a separate failure boundary.
    lifecycle_manager = Node(
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
    )

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=_launch_servers),
        lifecycle_manager,
    ])

# 配置	决定什么	具体含义
# use_composition=False	进程层面	每个节点是独立的可执行文件，各自启动一个 OS 进程，节点之间只能通过 DDS 走进程间通信（IPC）
# use_composition=True	进程层面	所有节点编译成共享库，加载到同一个容器进程里，节点之间可以走进程内零拷贝通信
# component_container（单线程版）	线程层面	容器内所有节点共享一个 SingleThreadedExecutor，即整个容器进程只有 1 条执行线程，所有回调串行执行
# component_container_mt	线程层面	容器内所有节点共享一个 MultiThreadedExecutor，即一个线程池（默认 = CPU 核心数），所有节点的回调从同一个池里取线程执行
# component_container_isolated	线程层面	每个组件有自己专属的 SingleThreadedExecutor + 专属线程，一个节点阻塞不会影响其他节点
