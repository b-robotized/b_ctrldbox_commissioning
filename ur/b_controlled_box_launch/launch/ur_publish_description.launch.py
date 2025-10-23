import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription, Condition
from launch.event_handlers import (
    OnProcessIO,
    OnProcessExit
)
from launch.actions import (
    RegisterEventHandler, 
    LogInfo, 
    SetLaunchConfiguration, 
    DeclareLaunchArgument,
    Shutdown,
    ExecuteProcess
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def on_ctrlx_runtime_config_dir_capture_success(event):
    """
    Callback triggered when the control box runtime config dir prefix is received from the topic.
    This function will construct the robot description and return the robot_state_publisher which depends on that path.
    """   
    captured_prefix = event.text.decode().strip()

    LogInfo(msg=f"Captured ctrlX runtime dir path prefix: '{captured_prefix}'")

    script_filename = PathJoinSubstitution([captured_prefix, "external_control.urscript"])
    input_recipe_filename = PathJoinSubstitution([captured_prefix, "rtde_input_recipe.txt"])
    output_recipe_filename = PathJoinSubstitution([captured_prefix, "rtde_output_recipe.txt"])

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            LaunchConfiguration("description_file"),
            " ",
            "robot_ip:=",
            LaunchConfiguration("robot_ip"),
            " ",
            "joint_limit_params:=",
            LaunchConfiguration("joint_limit_params_file"),
            " ",
            "kinematics_params:=",
            LaunchConfiguration("kinematics_params_file"),
            " ",
            "physical_params:=",
            LaunchConfiguration("physical_params_file"),
            " ",
            "visual_params:=",
            LaunchConfiguration("visual_params_file"),
            " ",
            "safety_limits:=",
            LaunchConfiguration("safety_limits"),
            " ",
            "safety_pos_margin:=",
            LaunchConfiguration("safety_pos_margin"),
            " ",
            "safety_k_position:=",
            LaunchConfiguration("safety_k_position"),
            " ",
            "name:=",
            LaunchConfiguration("ur_type"),
            " ",
            "script_filename:=",
            script_filename,
            " ",
            "input_recipe_filename:=",
            input_recipe_filename,
            " ",
            "output_recipe_filename:=",
            output_recipe_filename,
            " ",
            "tf_prefix:=",
            LaunchConfiguration("tf_prefix"),
            " ",
            "use_mock_hardware:=",
            LaunchConfiguration("use_mock_hardware"),
            " ",
            "mock_sensor_commands:=",
            LaunchConfiguration("mock_sensor_commands"),
            " ",
            "headless_mode:=",
            LaunchConfiguration("headless_mode"),
            " ",
            "use_tool_communication:=",
            LaunchConfiguration("use_tool_communication"),
            " ",
            "tool_parity:=",
            LaunchConfiguration("tool_parity"),
            " ",
            "tool_baud_rate:=",
            LaunchConfiguration("tool_baud_rate"),
            " ",
            "tool_stop_bits:=",
            LaunchConfiguration("tool_stop_bits"),
            " ",
            "tool_rx_idle_chars:=",
            LaunchConfiguration("tool_rx_idle_chars"),
            " ",
            "tool_tx_idle_chars:=",
            LaunchConfiguration("tool_tx_idle_chars"),
            " ",
            "tool_device_name:=",
            LaunchConfiguration("tool_device_name"),
            " ",
            "tool_tcp_port:=",
            LaunchConfiguration("tool_tcp_port"),
            " ",
            "tool_voltage:=",
            LaunchConfiguration("tool_voltage"),
            " ",
            "reverse_ip:=",
            LaunchConfiguration("reverse_ip"),
            " ",
            "script_command_port:=",
            LaunchConfiguration("script_command_port"),
            " ",
            "reverse_port:=",
            LaunchConfiguration("reverse_port"),
            " ",
            "script_sender_port:=",
            LaunchConfiguration("script_sender_port"),
            " ",
            "trajectory_port:=",
            LaunchConfiguration("trajectory_port"),
            " ",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # The callback returns a list of actions to be executed by the launch system
    return [
        SetLaunchConfiguration('base_path_prefix', captured_prefix),
        LogInfo(msg="Runtime config prefix received. Launching robot_state_publisher..."),
        robot_state_publisher_node
    ]

def generate_launch_description():

    ctrlx_runtime_config_dir_capture = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'b_controlled_box_launch',
            'listener_string',
            '--topic', '/b_controlled_box_cm/ctrlx/runtime_config_dir',
            '--timeout', '30'
        ],
        name='ctrlx_runtime_config_dir_capture',
        emulate_tty=True # output to a separate, virtual terminal
    )
    ctrlx_runtime_config_dir_capture_handler = RegisterEventHandler(
        event_handler=OnProcessIO(
            target_action=ctrlx_runtime_config_dir_capture,
            on_stdout=on_ctrlx_runtime_config_dir_capture_success
        )
    )

    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Typo/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur7e",
                "ur10",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur15",
                "ur20",
                "ur30",
            ],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'runtime_config_dir',
            default_value='',
            description='Base path prefix for runtime_configs directory on ControlBox.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip", description="IP address by which the robot can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "joint_limit_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur_description"),
                    "config",
                    LaunchConfiguration("ur_type"),
                    "joint_limits.yaml",
                ]
            ),
            description="Config file containing the joint limits (e.g. velocities, positions) of the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur_description"),
                    "config",
                    LaunchConfiguration("ur_type"),
                    "default_kinematics.yaml",
                ]
            ),
            description="The calibration configuration of the actual robot used.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "physical_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur_description"),
                    "config",
                    LaunchConfiguration("ur_type"),
                    "physical_parameters.yaml",
                ]
            ),
            description="Config file containing the physical parameters (e.g. masses, inertia) of the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "visual_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur_description"),
                    "config",
                    LaunchConfiguration("ur_type"),
                    "visual_parameters.yaml",
                ]
            ),
            description="Config file containing the visual parameters (e.g. meshes) of the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "urdf", "ur.urdf.xacro"]
            ),
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="tf_prefix of the joint names, useful for "
            "multi-robot setup. If changed, also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors used for simple simulations. "
            "Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Initially loaded robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_dashboard_client",
            default_value="true",
            description="Launch Dashboard Client?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_tool_communication",
            default_value="false",
            description="Only available for e series!",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_parity",
            default_value="0",
            description="Parity configuration for serial communication. Only effective, if "
            "use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_baud_rate",
            default_value="115200",
            description="Baud rate configuration for serial communication. Only effective, if "
            "use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_stop_bits",
            default_value="1",
            description="Stop bits configuration for serial communication. Only effective, if "
            "use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_rx_idle_chars",
            default_value="1.5",
            description="RX idle chars configuration for serial communication. Only effective, "
            "if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tx_idle_chars",
            default_value="3.5",
            description="TX idle chars configuration for serial communication. Only effective, "
            "if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_device_name",
            default_value="/tmp/ttyUR",
            description="File descriptor that will be generated for the tool communication device. "
            "The user has be be allowed to write to this location. "
            "Only effective, if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tcp_port",
            default_value="54321",
            description="Remote port that will be used for bridging the tool's serial device. "
            "Only effective, if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_voltage",
            default_value="0",  # 0 being a conservative value that won't destroy anything
            description="Tool voltage that will be setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_ip",
            default_value="0.0.0.0",
            description="IP that will be used for the robot controller to communicate back to the driver.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_command_port",
            default_value="50004",
            description="Port that will be opened to forward URScript commands to the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_port",
            default_value="50001",
            description="Port that will be opened to send cyclic instructions from the driver to the robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_sender_port",
            default_value="50002",
            description="The driver will offer an interface to query the external_control URScript on this port.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "trajectory_port",
            default_value="50003",
            description="Port that will be opened for trajectory control.",
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            ctrlx_runtime_config_dir_capture_handler,
            ctrlx_runtime_config_dir_capture,
        ]
    )
