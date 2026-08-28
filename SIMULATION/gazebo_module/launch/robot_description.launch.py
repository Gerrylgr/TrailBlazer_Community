import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取 URDF 文件路径
    urdf_path = get_package_share_directory("gazebo_module")
    default_model_path = urdf_path + "/urdf/model.urdf"

    # 声明启动参数
    action_declare_urdf_path = DeclareLaunchArgument(
        name='strutt_model',
        default_value=str(default_model_path),
        description="机器人的 URDF 文件路径"
    )

    # 使用 xacro 转换为 URDF
    robot_description = {
        'robot_description': launch_ros.parameter_descriptions.ParameterValue(
            Command(['cat ', LaunchConfiguration('strutt_model')]), value_type=str
        )
    }

    # robot_state_publisher 节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
    )

    # joint_state_publisher 节点
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    # rviz2 节点
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
    )

    # 返回 LaunchDescription
    return LaunchDescription([
        action_declare_urdf_path,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
