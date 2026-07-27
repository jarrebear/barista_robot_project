import os
from ament_index_python.packages import (get_package_prefix, get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, TimerAction)
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter, Node

import xacro

# ROS2 Launch System will look for this function definition #
def generate_launch_description():

    # Get Package Directory #
    pkg_barista_robot_description = get_package_share_directory('barista_robot_description')
    gz_sim_pkg = get_package_share_directory("ros_gz_sim")

    # Set the Path to Robot Mesh Models for Loading in Gazebo Sim #
    # NOTE: Do this BEFORE launching Gazebo Sim #
    install_dir_path_description = (get_package_prefix('barista_robot_description') + "/share")
    # gazebo_models_path = os.path.join(pkg_box_bot_gazebo, "models")
    description_meshes_path = os.path.join(pkg_barista_robot_description, "meshes")
    gazebo_resource_paths = [install_dir_path_description,  description_meshes_path] #gazebo_models_path]
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        for resource_path in gazebo_resource_paths:
            if resource_path not in os.environ["GZ_SIM_RESOURCE_PATH"]:
                os.environ["GZ_SIM_RESOURCE_PATH"] += (':' + resource_path)
    else:
        os.environ["GZ_SIM_RESOURCE_PATH"] = (':'.join(gazebo_resource_paths))

    # --------------------------------------------------
    # Launch Gazebo world
    # --------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': [
            '-r ',  # <-- start unpaused
            PathJoinSubstitution([pkg_barista_robot_description, 'worlds', 'barista_empty.world'])
        ]}.items(),
    )

    # convert XACRO file into URDF
    xacro_file = os.path.join(pkg_barista_robot_description, 'xacro', 'barista_robot_model.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    # --------------------------------------------------
    # Robot State Publisher
    # --------------------------------------------------

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        output="screen",
        parameters=[params]
    )

    # --------------------------------------------------
    # Spawn arguments
    # --------------------------------------------------

    declare_spawn_model_name = DeclareLaunchArgument(
        "model_name",
        default_value="barista_robot",
        description="Model spawn name",
    )

    declare_spawn_x = DeclareLaunchArgument(
        "x",
        default_value="0.0",
        description="Spawn X position",
    )

    declare_spawn_y = DeclareLaunchArgument(
        "y",
        default_value="0.0",
        description="Spawn Y position",
    )

    declare_spawn_z = DeclareLaunchArgument(
        "z",
        default_value="0.2",
        description="Spawn Z position",
    )

    declare_spawn_yaw = DeclareLaunchArgument(
        "yaw",
        default_value="0.0",
        description="Spawn yaw angle",
    )


    # --------------------------------------------------
    # Spawn robot into Gazebo
    # --------------------------------------------------

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="barista_robot_spawn",
        output="screen",
        arguments=[
            "-name",
            LaunchConfiguration("model_name"),

            "-allow_renaming",
            "true",

            "-topic",
            "robot_description",

            "-x",
            LaunchConfiguration("x"),

            "-y",
            LaunchConfiguration("y"),

            "-z",
            LaunchConfiguration("z"),

            "-Y",
            LaunchConfiguration("yaw"),
        ],
    )


    # Delay spawn until Gazebo is running
    delayed_spawn = TimerAction(
        period=2.0,
        actions=[
            gz_spawn_entity
        ],
    )


    # RVIZ Configuration
    rviz_config_dir = os.path.join(
        pkg_barista_robot_description,
        "rviz",
        "urdf_vis.rviz"
        )

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    # --------------------------------------------------
    # ROS Gazebo Bridge
    # --------------------------------------------------

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        arguments=[
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[gz.msgs.Clock",
            "/cmd_vel" + "@geometry_msgs/msg/Twist" + "@gz.msgs.Twist",
            "/tf" + "@tf2_msgs/msg/TFMessage" + "[gz.msgs.Pose_V",
            "/odom" + "@nav_msgs/msg/Odometry" + "[gz.msgs.Odometry",
            "/world/demo/model/barista_robot/joint_state" + "@sensor_msgs/msg/JointState" + "[gz.msgs.Model",
            "/laser_scan" + "@sensor_msgs/msg/LaserScan" + "[gz.msgs.LaserScan",
        ],
        remappings=[
            ("/world/demo/model/barista_robot/joint_state", "/joint_states"),
            ("/laser_scan", "/scan"),
        ],
        output="screen",
    )

    # --------------------------------------------------
    # Launch description
    # --------------------------------------------------

    return LaunchDescription(
        [

            SetParameter(
                name="use_sim_time",
                value=True,
            ),

            # Arguments
            declare_spawn_model_name,
            declare_spawn_x,
            declare_spawn_y,
            declare_spawn_z,
            declare_spawn_yaw,

            # Simulator
            gz_sim,

            # Robot description
            robot_state_publisher_node,

            # Spawn robot
            delayed_spawn,
            
            # Load rviz
            rviz_node,

            # Gazebo bridge
            gz_bridge

        ]
    )