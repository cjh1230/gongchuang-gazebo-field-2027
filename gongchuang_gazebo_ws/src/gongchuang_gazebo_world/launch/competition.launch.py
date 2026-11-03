from pathlib import Path
import shlex
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    pkg=get_package_share_directory('gongchuang_gazebo_world')
    def run_gazebo(context):
        layout=LaunchConfiguration('layout').perform(context)
        fieldroot=LaunchConfiguration('field_root').perform(context)
        flags=shlex.split(LaunchConfiguration('gz_flags').perform(context))
        return [ExecuteProcess(cmd=['gz','sim','-r',str(Path(fieldroot)/'worlds'/(layout+'.sdf')),*flags],output='screen')]
    return LaunchDescription([
        DeclareLaunchArgument('layout', default_value='random_seeded',
            choices=['competition','empty_obstacles','single_obstacle','blocked_corridor','audit_views','audit_stacking','batch1_coarse','batch2_ready','batch2_coarse','stacking_demo','random_seeded','random_batch1_coarse','random_batch2_ready','random_batch2_coarse','random_stacking_demo','audit_random','audit_random_stacking','obstacles_0','obstacles_1','obstacles_2','obstacles_3','stress_4','stress_5','stress_6','central_blocked','audit_central_blocked','audit_stress_6']),
        DeclareLaunchArgument('field_root',default_value=pkg),
        DeclareLaunchArgument('gz_flags',default_value=''),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',PathJoinSubstitution([LaunchConfiguration('field_root'),'models'])),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',str(Path(get_package_prefix('gongchuang_gazebo_world'))/'lib')),
        SetEnvironmentVariable('GZ_PARTITION','gongchuang_local'),
        SetEnvironmentVariable('QT_QPA_PLATFORM','xcb'),
        OpaqueFunction(function=run_gazebo),
        Node(package='ros_gz_bridge',executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],output='screen'),
        Node(package='tf2_ros',executable='static_transform_publisher',
            arguments=['--x','0','--y','0','--z','0','--roll','0','--pitch','0','--yaw','0',
                       '--frame-id','world','--child-frame-id','map']),
    ])
