import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
from ament_index_python.packages import get_package_share_directory
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Uncomment this section to use the real robot

    # robot_ip = LaunchConfiguration('robot_ip')
    # hw_ns = LaunchConfiguration('hw_ns', default='ufactory')
    
    # # robot moveit realmove launch
    # # xarm_moveit_config/launch/_robot_moveit_realmove.launch.py
    # robot_moveit_realmove_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_realmove.launch.py'])),
    #     launch_arguments={
    #         'robot_ip': robot_ip,
    #         'dof': '6',
    #         'robot_type': 'uf850',
    #         'hw_ns': hw_ns,
    #         'no_gui_ctrl': 'false',
    #         'add_gripper': 'true',

    #     }.items(),
    # )


    # Load the robot configuration Fake Robot

    moveit_config = (
        MoveItConfigsBuilder(
            controllers_name="fake_controllers",
            ros2_control_plugin="uf_robot_hardware/UFRobotFakeSystemHardware",
            context=LaunchContext(),
            robot_type="uf850",
            dof=6,
            add_gripper = "true"
        )
        .robot_description()
        .trajectory_execution(file_path="config/uf850/fake_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--log-level", "debug"],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("xarm_moveit_config") + "/rviz/moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("xarm_controller"),
        "config",
        "uf850_controllers.yaml",
    )


    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["uf850_traj_controller", "-c", "/controller_manager"],
    )

   

    go_to_action_server = Node(
        package="astro_actions",
        executable="go_to_action_server",
        output="screen",
        parameters=[],
        arguments=["--log-level", "debug"],
    )
    wait_for_op_cmd_action_server = Node(
        package="astro_actions",
        executable="wait_for_cmd_action_server.py",
        output="screen",
        parameters=[],
        arguments=["--log-level", "debug"],
    )
    state_machine = Node(
        package="astro_state_machine",
        executable="astro_state_machine",
        output="screen",
        parameters=[],
        arguments=["--log-level", "debug"],
    )

    image_stream_publisher = Node(
        package="astro_image_publisher",
        executable="astro_img_publisher",
        output="screen",
        parameters=[],
        arguments=["--log-level", "debug"],
    )

    aruco_publisher = Node(
        package="aruco_publisher",
        executable="aruco_publisher",
        output="screen",
        parameters=[],
        arguments=["--log-level", "debug"],
    )

    yasmin_viewer = Node(
        package="yasmin_viewer",
        executable="yasmin_viewer_node",
        output="screen",
        parameters=[],
        arguments=["--log-level", "debug"],
    )
    

    tf_listener = Node(
        package="robotic_arm_tf",
        executable="tf_listener",
        output="screen",
        parameters=[],
        arguments=["--log-level", "debug"],
    )

   
    tf_broadcast = Node(
        package="robotic_arm_tf",
        executable="tf_broadcast",
        output="screen",
        parameters=[],
        arguments=["--log-level", "debug"],
    )
    # RViz
    return LaunchDescription(
        [
            # robot_moveit_realmove_launch,
            go_to_action_server,
            wait_for_op_cmd_action_server,
            state_machine,
            image_stream_publisher,
            aruco_publisher,  
            yasmin_viewer,  
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            tf_listener,
            tf_broadcast      
        ]
    )