import os
from ament_index_python.packages import (get_package_prefix, get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction)
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter, Node
from launch.conditions import IfCondition

import xacro

# ROS2 Launch System will look for this function definition #
def generate_launch_description():

    # --------------------------------------------------
    # Launch arguments
    # --------------------------------------------------

    declare_include_laser = DeclareLaunchArgument(
        "include_laser",
        default_value="true",
        description="Whether to include the laser scanner in the robot description",
    )

    declare_robot_color = DeclareLaunchArgument(
        "robot_color",
        default_value="blue",
        description="Color of the robot, see .xacro file for details on which colors are available",
    )

    declare_spawn_robot_name = DeclareLaunchArgument(
        "robot_name",
        default_value="barista_robot",
        description="Robot spawn name",
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
    # Package paths and gazebo resources
    # --------------------------------------------------

    pkg_barista_robot_description = get_package_share_directory('barista_robot_description')
    gz_sim_pkg = get_package_share_directory("ros_gz_sim")

    install_dir_path_description = (get_package_prefix('barista_robot_description') + "/share")
    description_meshes_path = os.path.join(pkg_barista_robot_description, "meshes")
    gazebo_resource_paths = [install_dir_path_description, description_meshes_path]
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
            '-r ',
            PathJoinSubstitution([pkg_barista_robot_description, 'worlds', 'barista_empty.world'])
        ]}.items(),
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
            "-name", LaunchConfiguration("robot_name"),
            "-allow_renaming", "true",
            "-topic", ["/", LaunchConfiguration("robot_name"), "/robot_description"],
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
    )

    delayed_spawn = TimerAction(
        period=2.0,
        actions=[gz_spawn_entity]
    )

    # --------------------------------------------------
    # RVIZ
    # --------------------------------------------------

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
    # xacro processing + robot_state_publisher
    # --------------------------------------------------

    def launch_setup(context, *args, **kwargs):

        # Resolve launch arguments
        robot_name_value = LaunchConfiguration("robot_name").perform(context)
        robot_color_value = LaunchConfiguration("robot_color").perform(context)
        include_laser_value = LaunchConfiguration("include_laser").perform(context)

        xacro_file = os.path.join(
            pkg_barista_robot_description, 'xacro', 'barista_robot_model.urdf.xacro'
        )

        # -----------------------------
        # Generate robot description
        # -----------------------------

        robot_description_content = xacro.process_file(
            xacro_file,
            mappings={
                "include_laser": include_laser_value,
                "robot_color": robot_color_value,
                "robot_name": robot_name_value,
            },
        ).toxml()

        # -----------------------------
        # Robot State Publisher
        # -----------------------------

        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_node',
            namespace=robot_name_value,
            parameters=[{
                "robot_description": robot_description_content,
                "use_sim_time": True,
                "frame_prefix": robot_name_value + "/",
            }],
            output="screen",
        )

        # -----------------------------
        # Joint state bridge
        # -----------------------------

        joint_state_gz_topic = f"/{robot_name_value}/joint_states"

        joint_state_ros_topic = (
            f"/{robot_name_value}/joint_states"
        )

        gz_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_bridge",
            arguments=[
                "/clock" + "@rosgraph_msgs/msg/Clock" + "[gz.msgs.Clock",
                f"/{robot_name_value}/cmd_vel" + "@geometry_msgs/msg/Twist" + "@gz.msgs.Twist",
                "/tf" + "@tf2_msgs/msg/TFMessage" + "[gz.msgs.Pose_V",
                f"/{robot_name_value}/odom" + "@nav_msgs/msg/Odometry" + "[gz.msgs.Odometry",

                joint_state_gz_topic
                    + "@sensor_msgs/msg/JointState"
                    + "[gz.msgs.Model",
            ],
            remappings=[
                (joint_state_gz_topic, joint_state_ros_topic),
            ],
            output="screen",
        )

        # Laser bridge
        laser_gz_topic = f"/{robot_name_value}/laser_scan"
        laser_ros_topic = f"/{robot_name_value}/scan"

        gz_bridge_laser = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_bridge_laser",
            condition=IfCondition(LaunchConfiguration("include_laser")),
            arguments=[
                laser_gz_topic
                + "@sensor_msgs/msg/LaserScan"
                + "[gz.msgs.LaserScan",
            ],
            remappings=[
                (laser_gz_topic, laser_ros_topic),
            ],
            output="screen",
        )

        return [
            robot_state_publisher_node,
            gz_bridge,
            gz_bridge_laser,
        ]

    # --------------------------------------------------
    # Launch description
    # --------------------------------------------------

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),

            # Arguments
            declare_include_laser,
            declare_spawn_robot_name,
            declare_robot_color,
            declare_spawn_x,
            declare_spawn_y,
            declare_spawn_z,
            declare_spawn_yaw,

            # Simulator
            gz_sim,

            # Robot description (deferred until include_laser is resolved)
            OpaqueFunction(function=launch_setup),

            # Spawn robot
            delayed_spawn,

            # Load rviz
            rviz_node,

        ]
    )