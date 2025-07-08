import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # --- 第1步：获取包路径 ---
    robot_navigation2_dir = get_package_share_directory('robot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # --- 第2步：定义默认文件路径 (直接用字符串) ---
    default_map_path = os.path.join(robot_navigation2_dir, 'maps', 'room.yaml')
    default_params_path = os.path.join(robot_navigation2_dir, 'config', 'nav2_params.yaml')
    default_rviz_config_path = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # --- 第3步：声明启动参数 (DeclareLaunchArgument) ---
    # 在这里为参数提供一个具体的默认值 (字符串)
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_path,
        description='Full path to param file to load')

    # --- 第4步：使用LaunchConfiguration引用已声明的参数 ---
    # LaunchConfiguration的作用是在运行时“获取”参数的值
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_path = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    # --- 第5步：构建启动描述 ---
    bringup_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        # 将获取到的参数值传递给 bringup_launch.py
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items(),
    )

    start_rviz_cmd = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    # --- 第6步：返回完整的LaunchDescription ---
    return launch.LaunchDescription([
        # 必须先声明参数
        declare_use_sim_time_cmd,
        declare_map_cmd,
        declare_params_file_cmd,

        # 然后再执行使用这些参数的动作
        bringup_nav2_cmd,
        start_rviz_cmd,
    ])