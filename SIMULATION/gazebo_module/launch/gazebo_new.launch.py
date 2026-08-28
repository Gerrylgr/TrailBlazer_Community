import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # 默认路径
    robot_name_in_model = "fishbot"
    urdf_tutorial_path = get_package_share_directory('gazebo_module')
    rviz_config_file = os.path.join(urdf_tutorial_path, 'config', 'rviz', 'my_config.rviz')
    default_model_path = urdf_tutorial_path + '/urdf/fishbot/fishbot.urdf.xacro'
    default_world_path = urdf_tutorial_path + '/world/hospital.world'

    # urdf 路径
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),description='URDF 的绝对路径')
    # 转换 xacro 并声明为 ROS 参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')]), value_type=str)
  	
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # 通过 IncludeLaunchDescription 包含另外一个 launch 文件
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py']),
        launch_arguments=[('world', default_world_path),('verbose','true')]
    )
    # 请求 Gazebo 加载机器人
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        parameters=[{'use_sim_time': True}],
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name_in_model, 
                   '-x', '1.0',
                    '-y', '2.0',
                    '-z', '0.07',
                    '-Y', '0.0'])
    
    rviz_node_cmd = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        name='trailblazer_rviz2',
        output='screen'
    )
    
    # 加载并激活 fishbot_joint_state_broadcaster
    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'fishbot_joint_state_broadcaster'],
        output='screen'
    )

    # 加载并激活 fishbot_effort_controller 控制器
    load_fishbot_effort_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','fishbot_effort_controller'], 
        output='screen')
    
    load_fishbot_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','fishbot_diff_drive_controller'], 
        output='screen')
    
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node,
        # 事件动作，当加载机器人结束后执行    
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=spawn_entity_node,
        #         on_exit=[load_joint_state_controller],)
        #     ),
        # 事件动作，load_fishbot_diff_drive_controller
        # launch.actions.RegisterEventHandler(
        # event_handler=launch.event_handlers.OnProcessExit(
        #     target_action=load_joint_state_controller,
        #     on_exit=[load_fishbot_diff_drive_controller],)          # 使用现成的两轮差速插件
        #     ),
        rviz_node_cmd
    ])
