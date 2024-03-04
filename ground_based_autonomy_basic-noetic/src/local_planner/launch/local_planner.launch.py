from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取参数文件的路径
    params_file_path = os.path.join(
        get_package_share_directory('local_planner'),
        'config',
        'params.yaml'
    )
    
    # 定义要包含的其他launch文件
    # ps3_launch_file = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         get_package_share_directory('ps3joy'),
    #         '/launch/ps3.launch.py'
    #     ])
    # )
    
    # 定义节点
    local_planner_node = Node(
        package='local_planner',
        executable='localPlanner',
        name='localPlanner',
        output='screen',
        parameters=[params_file_path],
    )
    
    path_follower_node = Node(
        package='local_planner',
        executable='pathFollower',
        name='pathFollower',
        output='screen',
        parameters=[params_file_path],
    )
    
    # 注意：以下是一个示例，静态转换发布器在ROS2中也有一些变化
    vehicle_trans_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "sensor", "vehicle"],
    )
    
    return LaunchDescription([
        # ps3_launch_file,
        local_planner_node,
        path_follower_node,
        vehicle_trans_publisher_node,
    ])