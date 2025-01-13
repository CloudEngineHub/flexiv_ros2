from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def generate_launch_description():
    rizon_type_param_name = "rizon_type"
    robot_sn_param_name = "robot_sn"
    start_rviz_param_name = "start_rviz"
    load_gripper_param_name = "load_gripper"
    gripper_name_param_name = "gripper_name"
    use_fake_hardware_param_name = "use_fake_hardware"
    fake_sensor_commands_param_name = "fake_sensor_commands"
    robot_controller_param_name = "robot_controller"

    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            rizon_type_param_name,
            description="Type of the Flexiv Rizon robot.",
            default_value="rizon4",
            choices=["rizon4", "rizon4s", "rizon10", "rizon10s"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            robot_sn_param_name,
            description="Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            start_rviz_param_name,
            default_value="true",
            description="Start RViz automatically with the launch file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            load_gripper_param_name,
            default_value="false",
            description="Flag to load the Flexiv Grav gripper as the end-effector of the robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            gripper_name_param_name,
            default_value="Flexiv-GN01",
            description="Full name of the gripper to be controlled, can be found in Flexiv Elements -> Settings -> Device",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            use_fake_hardware_param_name,
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            fake_sensor_commands_param_name,
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_fake_hardware' parameter is true.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            robot_controller_param_name,
            default_value="rizon_arm_controller",
            description="Robot controller to start. Available: forward_position_controller, rizon_arm_controller, joint_impedance_controller.",
        )
    )

    # Initialize Arguments
    rizon_type = LaunchConfiguration(rizon_type_param_name)
    robot_sn = LaunchConfiguration(robot_sn_param_name)
    start_rviz = LaunchConfiguration(start_rviz_param_name)
    load_gripper = LaunchConfiguration(load_gripper_param_name)
    gripper_name = LaunchConfiguration(gripper_name_param_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_param_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_param_name)
    robot_controller = LaunchConfiguration(robot_controller_param_name)

    # Get URDF via xacro
    flexiv_urdf_xacro = PathJoinSubstitution(
        [FindPackageShare("flexiv_description"), "urdf", "rizon.urdf.xacro"]
    )

    # Get URDF via xacro
    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                flexiv_urdf_xacro,
                " ",
                "robot_sn:=",
                robot_sn,
                " ",
                "name:=",
                "rizon",
                " ",
                "rizon_type:=",
                rizon_type,
                " ",
                "load_gripper:=",
                load_gripper,
                " ",
                "gripper_name:=",
                gripper_name,
                " ",
                "use_fake_hardware:=",
                use_fake_hardware,
                " ",
                "fake_sensor_commands:=",
                fake_sensor_commands,
            ]
        ),
        value_type=str,
    )

    robot_description = {"robot_description": robot_description_content}

    # RViZ
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("flexiv_description"), "rviz", "view_rizon.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(start_rviz),
    )

    # Robot controllers
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("flexiv_bringup"), "config", "rizon_controllers.yaml"]
    )

    # Controller Manager
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers, {"robot_sn": robot_sn}],
        remappings=[("joint_states", "flexiv_arm/joint_states")],
        output="both",
    )

    # Joint state publisher
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            {
                "source_list": [
                    "flexiv_arm/joint_states",
                    "flexiv_gripper_node/gripper_joint_states",
                ],
                "rate": 30,
            }
        ],
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Run robot controller
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller, "--controller-manager", "/controller_manager"],
    )

    # Run joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Run Flexiv robot states broadcaster
    flexiv_robot_states_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["flexiv_robot_states_broadcaster"],
        parameters=[{"robot_sn": robot_sn}],
        condition=UnlessCondition(use_fake_hardware),
    )

    # Include gripper launch file
    load_gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("flexiv_gripper"),
                    "launch",
                    "flexiv_gripper.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_sn": robot_sn,
            "gripper_name": gripper_name,
            "use_fake_hardware": use_fake_hardware,
        }.items(),
        condition=IfCondition(load_gripper),
    )

    # Run gpio controller
    gpio_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gpio_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        )
    )

    nodes = [
        ros2_control_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        flexiv_robot_states_broadcaster_spawner,
        load_gripper_launch,
        gpio_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
