# 文件名：robot_spawn.launch.py (或者您的实际文件名)
# --- 导入所需模块 ---
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory  
import os
from launch_ros.actions import Node
import launch_ros.parameter_descriptions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument  # 确保导入这个

def generate_launch_description():
    # 1. 声明并获取 use_sim_time 参数
    #    这将允许我们从命令行控制它，并为所有节点提供统一的来源。
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ... 您原来的路径定义保持不变 ...
    urdf_package_path = get_package_share_directory('my_robot_description') 
    default_urdf_path = os.path.join(urdf_package_path, 'urdf', 'robot.urdf.xacro')  
    default_gazebo_wolrd_path = os.path.join(urdf_package_path, 'world', 'custom_room.world')
    
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=default_urdf_path,
        description='加载的模型文件路径'
    )

    command_result = launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')])  
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(command_result, value_type=str)   

    # 2. 修改 robot_state_publisher 节点
    #    为它添加 use_sim_time 参数。
    action_robot_state_pubisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',    
        parameters=[{'robot_description': robot_description_value,
                     'use_sim_time': use_sim_time}]  # <--- 修改这里
    )

    # ... action_load_joint_control 和 action_load_diff_driver_control 保持不变 ...
    # 这两个是通过ros2_control插件控制的，我们将在第二步中配置它。
    action_load_joint_control = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller robot_joint_state_broadcaster --set-state active'.split(),
        output='screen'
    )
    action_load_diff_driver_control = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller robot_diff_driver_controller --set-state active'.split(),
        output='screen'
    )
    
    # 3. 修改 Gazebo 的启动
    #    确保 Gazebo 被告知要发布仿真时钟。
    #    gazebo_ros 的 launch 文件会自动处理 use_sim_time 参数。
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments=[('world', default_gazebo_wolrd_path), 
                          ('verbose', 'true')]
    )

    # 4. 修改 spawn_entity 节点 (可选但推荐)
    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', 
                '-entity', 'robot',
                '-x', '1.0',
                '-y', '0.0',
                '-z', '0.0'],
        # 虽然它对时间不敏感，但保持一致性是好习惯。
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ... 您的 GroupAction 保持不变 ...
    action_group = launch.actions.GroupAction([
        launch.actions.TimerAction(period = 0.0, actions = [action_declare_arg_mode_path]),
        launch.actions.TimerAction(period = 0.2, actions = [action_robot_state_pubisher]),
        launch.actions.TimerAction(period = 0.4, actions = [action_launch_gazebo]),
        launch.actions.TimerAction(period = 0.8, actions = [action_spawn_entity]),
        launch.actions.TimerAction(period = 1.0, actions = [action_load_joint_control]),
        launch.actions.TimerAction(period = 1.2, actions = [action_load_diff_driver_control]),
    ])    

    # 5. 在最外层返回 LaunchDescription，并包含参数声明
    return launch.LaunchDescription([
        # 这是全局开关，必须在这里声明！
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        action_group
    ])