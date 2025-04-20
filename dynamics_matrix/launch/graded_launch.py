from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    pkg_dynamics_matrix = FindPackageShare('dynamics_matrix')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([pkg_dynamics_matrix, 'models'])
        ),
        SetEnvironmentVariable(
            'GZ_SIM_PLUGIN_PATH',
            PathJoinSubstitution([pkg_dynamics_matrix, 'plugins'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [PathJoinSubstitution([pkg_dynamics_matrix, 'worlds/graded_buoyancy_world.sdf'])],
                'on_exit_shutdown': 'True'
            }.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                "/model/my_lrauv_modified/joint/horizontal_fins_joint/0/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double",
               # "/model/my_lrauv_modified/joint/vertical_fins_joint/0/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double",
                "/model/my_lrauv/joint/propeller_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double",
                "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            ],
            remappings=[
                ('/model/my_lrauv_modified/joint/horizontal_fins_joint/0/cmd_pos', 'my_lrauv_modified/horizontal/cmd_p'),
              # ('/model/my_lrauv_modified/joint/vertical_fins_joint/0/cmd_pos', 'my_lrauv_modified/vertical_fins/cmd_pos'),
                ('/model/my_lrauv/joint/propeller_joint/cmd_thrust', 'my_lrauv_modified/propeller/thrust'),
                ('/imu','/imu/submarine'),
                
                 
               


            ],
            output='screen'
        ),
    ])
