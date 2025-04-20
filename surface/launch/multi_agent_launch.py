from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    pkg_surface = FindPackageShare('surface')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    agents = [f'my_lrauv_{i+1}' for i in range(8)]
    agent_launches = []

    for agent in agents:
        agent_launches.append(
            GroupAction([
                PushRosNamespace(agent),
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    arguments=[
                        '/world/wamv_world/model/wamv/link/wamv/imu_wamv_link/sensor/imu_wamv_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                       '/world/wamv_world/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
                       '/model/wamv/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                       '/model/wamv/joint/left_engine_propeller_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                       '/model/wamv/joint/right_engine_propeller_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                       '/model/wamv/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                       '/debug/wind/speed@std_msgs/msg/Float32@gz.msgs.Float',
                       '/debug/wind/direction@std_msgs/msg/Float32@gz.msgs.Float',
                       '/world/wamv_anchor_world/model/wamv/model/wamv/model/wamv/link/wamv/base_link/sensor/front_camera_sensor/image@sensor_msgs/msg/Image@gz.msgs.Image',
                       '/world/wamv_anchor_world/model/wamv/model/wamv/model/wamv/link/wamv/base_link/sensor/front_camera_sensor/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                       '/world/wamv_anchor_world/model/wamv/model/wamv/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                       '/world/wamv_anchor_world/model/wamv/model/wamv/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                       
                        f'/model/{agent}/joint/propeller_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                        f'/model/{agent}/joint/horizontal_fins_cmd@std_msgs/msg/Float64@gz.msgs.Double',
                        f'/model/{agent}/joint/vertical_fins_cmd@std_msgs/msg/Float64@gz.msgs.Double',
                        f'/model/{agent}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                        f'/world/wamv_world/model/{agent}/link/base_link/sensor/magnetometer/magnetometer@sensor_msgs/msg/MagneticField@gz.msgs.Magnetometer',
                        f'/world/wamv_world/model/{agent}/link/base_link/sensor/air_pressure/air_pressure@sensor_msgs/msg/FluidPressure@gz.msgs.FluidPressure',
                        f'/{agent}/close_one_to_sonar_bottom@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                        f'/{agent}/close_one_to_sonar_top@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                        f'/{agent}/close_one_to_sonar_front@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
                    ],
                    remappings=[
                        ('/world/wamv_world/model/wamv/link/wamv/imu_wamv_link/sensor/imu_wamv_sensor/imu',
                            '/wamv/sensors/imu/imu/data'),
                                ('/world/wamv_world/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat',
                        '/wamv/sensors/gps/gps/fix'),
                        ('/model/wamv/joint/left_engine_propeller_joint/cmd_thrust',
                        '/wamv/thrusters/left/thrust'),
                        ('/model/wamv/joint/right_engine_propeller_joint/cmd_thrust',
                        '/wamv/thrusters/right/thrust'),
                        ('/model/wamv/odometry', '/wamv/ground_truth/odometry'),
                        ('/model/wamv/model/wamv/model/pose',
                        '/wamv/ground_truth/pose'),
                        ('/world/wamv_anchor_world/model/wamv/model/wamv/model/wamv/link/wamv/base_link/sensor/front_camera_sensor/image',
                        '/wamv/sensors/camera/front/image'),
                        ('/world/wamv_anchor_world/model/wamv/model/wamv/model/wamv/link/wamv/base_link/sensor/front_camera_sensor/camera_info',
                        '/wamv/sensors/camera/front/camera_info'),
                        ('/world/wamv_anchor_world/model/wamv/model/wamv/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan',
                        '/wamv/sensors/front_lidar/scan'),
                        ('/world/wamv_anchor_world/model/wamv/model/wamv/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan/points',
                        '/wamv/sensors/front_lidar/points'),
                        (f'/model/{agent}/joint/propeller_joint/cmd_thrust', f'/{agent}/propeller/thrust'),
                        (f'/model/{agent}/joint/horizontal_fins_cmd', f'/{agent}/submarine/horizontal/fin/pos'),
                        (f'/model/{agent}/joint/vertical_fins_cmd', f'/{agent}/submarine/vertical/fin/pos'),
                        (f'/model/{agent}/odometry', f'/{agent}/submarine/odometry'),
                        (f'/world/wamv_world/model/{agent}/link/base_link/sensor/magnetometer/magnetometer', f'/{agent}/magneto/meter'),
                        (f'/world/wamv_world/model/{agent}/link/base_link/sensor/air_pressure/air_pressure', f'/{agent}/air/press'),
                        (f'/{agent}/close_one_to_sonar_front', f'/{agent}/submarine/Laser_scan_front'),
                        (f'/{agent}/close_one_to_sonar_top', f'/{agent}/submarine/Laser_scan_top'),
                        (f'/{agent}/close_one_to_sonar_bottom', f'/{agent}/submarine/Laser_scan_bottom'),
                    ],
                    output='screen'
                )
            ])
        )

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', PathJoinSubstitution([pkg_surface, 'models'])
        ),
        SetEnvironmentVariable(
            'GZ_SIM_PLUGIN_PATH', PathJoinSubstitution([pkg_surface, 'plugins'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [PathJoinSubstitution([pkg_surface, 'worlds/multi_agent.sdf'])],
                'on_exit_shutdown': 'True'
            }.items(),
        ),
    ] + agent_launches)

