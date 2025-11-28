from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    pkg_surface = FindPackageShare('surface')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([pkg_surface, 'models'])
        ),
        SetEnvironmentVariable(
            'GZ_SIM_PLUGIN_PATH',
            PathJoinSubstitution([pkg_surface, 'plugins'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [PathJoinSubstitution([pkg_surface, 'worlds/control.sdf'])],
                'on_exit_shutdown': 'True'
            }.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/world/wamv_world/model/wamv/link/wamv/imu_wamv_link/sensor/imu_wamv_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
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
                        '/model/my_lrauv/joint/propeller_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                        '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                        '/world/wamv_world/model/my_lrauv_modified/link/base_link/sensor/magnetometer/magnetometer@sensor_msgs/msg/MagneticField@gz.msgs.Magnetometer',
                        '/world/wamv_world/model/my_lrauv_modified/link/base_link/sensor/air_pressure/air_pressure@sensor_msgs/msg/FluidPressure@gz.msgs.FluidPressure',    
                        '/wamv/left/thruster/joint/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
                        '/wamv/right/thruster/joint/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',                 
                       '/model/my_lrauv_modified/joint/horizontal_fins_cmd@std_msgs/msg/Float64@gz.msgs.Double',
                      '/model/my_lrauv_modified/joint/vertical_fins_cmd@std_msgs/msg/Float64@gz.msgs.Double',
                      '/model/my_lrauv_modified/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                      '/close_one_to_sonar_bottom@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                      '/close_one_to_sonar_top@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                       '/close_one_to_sonar_front@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
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
                ('/model/my_lrauv/joint/propeller_joint/cmd_thrust', 'my_lrauv_modified/propeller/thrust'),
                ('/imu','my_lrauv_modified/imu/submarine'),
                ('/world/wamv_world/model/my_lrauv_modified/link/base_link/sensor/magnetometer/magnetometer','my_lrauv_modified/magneto/meter'),
                ('/world/wamv_world/model/my_lrauv_modified/link/base_link/sensor/air_pressure/air_pressure','my_lrauv_modified/air/press'),
                ('/wamv/left/thruster/joint/cmd_pos','wamv/thrusters/left/angle' ),
                ( '/wamv/right/thruster/joint/cmd_pos','wamv/thrusters/right/angle' ),
               ('/model/my_lrauv_modified/joint/vertical_fins_cmd','my_lrauv_modified/submarine/yaw/fin/pos'),
                ('/model/my_lrauv_modified/joint/horizontal_fins_cmd','my_lrauv_modified/submarine/pitch/fin/pos'),
                ('/model/my_lrauv_modified/odometry','my_lrauv_modified/submarine/odometry'),
                ('/close_one_to_sonar_front','my_lrauv_modified/submarine/Laser_scan_front'),
                ( '/close_one_to_sonar_top','my_lrauv_modified/submarine/Laser_scan_top'),
                 ( '/close_one_to_sonar_bottom','my_lrauv_modified/submarine/Laser_scan_bottom'),

                 
               


            ],
            output='screen'
        ),
    ])
