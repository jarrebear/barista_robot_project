import os

from ament_index_python.packages import (
    get_package_share_directory,
    get_package_prefix,
)

from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
)

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter


def generate_launch_description():

    # --------------------------------------------------
    # Package paths
    # --------------------------------------------------

    package_name = "barista_robot_description"

    pkg_description = get_package_share_directory(package_name)
    pkg_prefix = get_package_prefix(package_name)

    gz_sim_pkg = get_package_share_directory("ros_gz_sim")


    # --------------------------------------------------
    # Gazebo resource paths
    # --------------------------------------------------

    gazebo_resource_paths = [
        pkg_prefix + "/share",
        os.path.join(pkg_description, "meshes"),
        os.path.join(pkg_description, "models"),
    ]

    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        for path in gazebo_resource_paths:
            if os.path.exists(path) and path not in os.environ["GZ_SIM_RESOURCE_PATH"]:
                os.environ["GZ_SIM_RESOURCE_PATH"] += ":" + path
    else:
        os.environ["GZ_SIM_RESOURCE_PATH"] = ":".join(
            [p for p in gazebo_resource_paths if os.path.exists(p)]
        )


    # --------------------------------------------------
    # Robot description
    # --------------------------------------------------

    urdf_file = os.path.join(
        pkg_description,
        "urdf",
        "barista_robot_model.urdf",
    )

    with open(urdf_file, "r") as file:
        robot_description = file.read()


    # --------------------------------------------------
    # Launch Gazebo world
    # --------------------------------------------------

    world_file = PathJoinSubstitution(
        [
            pkg_description,
            "worlds",
            "barista_empty.world",
        ]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gz_sim_pkg,
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": [
                "-r ",
                world_file,
            ]
        }.items(),
    )


    # --------------------------------------------------
    # Robot State Publisher
    # --------------------------------------------------

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": robot_description,
            }
        ],
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
    rviz_config_dir = os.path.join(get_package_share_directory(package_name), 'rviz', 'urdf_vis.rviz')


    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

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
            robot_state_publisher,

            # Spawn robot
            delayed_spawn,
            
            # Load rviz
            rviz_node,

        ]
    )
