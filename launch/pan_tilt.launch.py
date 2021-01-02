import launch
import launch_ros.actions

import os
from ament_index_python import get_package_share_directory
share_dir = get_package_share_directory('dynamixel_pantilt')
manager_params = os.path.join(
    share_dir, 'config', 'manager_params.yaml'
)
spawner_params = os.path.join(
    share_dir, 'config', 'spawner_params.yaml')
pantilt_xacro = os.path.join(
    share_dir, 'urdf', 'pan_tilt.urdf.xacro')

def generate_launch_description():
    manager = launch_ros.actions.Node(
        package = 'dynamixel_controllers',
        node_executable = 'controller_manager',
        output = 'screen',
        node_name = 'dynamixel_manager',
        parameters = [manager_params]
    )

    spawner = launch_ros.actions.Node(
        package = 'dynamixel_controllers',
        node_executable = 'controller_spawner',
        output = 'screen',
        node_name = 'dynamixel_controller_spawner',
        parameters = [spawner_params],
        arguments = [
            '--manager=dynamixel_controller_manager',
            '--port=pan_tilt',
            'pan_controller',
            'tilt_controller'
        ]
    )

    joint_state_publisher = launch_ros.actions.Node(
        package = 'dynamixel_pantilt',
        node_executable = 'dynamixel_joint_state_publisher',
        output = 'screen',
        node_name = 'joint_state_publisher'
    )

    import tempfile
    import xacro
    with tempfile.NamedTemporaryFile(mode='w', delete=False) as urdf_file:
        doc = xacro.process_file(xacro_path)
        robot_desc = doc.toprettyxml(indent='  ')
        urdf_file.write(robot_desc)
        
    robot_state_publisher = launch_ros.actions.Node(
        package = 'robot_state_publisher',
        node_executable = 'robot_state_publisher',
        output = 'screen',
        node_name = 'robot_state_publisher',
        parameters = [{
            'dynamixel_controllers': [
                'pan_controller',
                'tilt_controller'
            ]
        }],
        arguments = [urdf_file.name]
    )

    def on_exit(context, *args, **kwargs):
        os.remove(urdf_file.name)
        
    register_on_exit = launch.actions.RegisterEventHandler(
        event_handler = launch.event_handlers.OnProcessExit(
            target_action = robot_state_publisher,
            on_exit = [launch.actions.OpaqueFunction(function=on_exit)],
        )
    )
    
    return launch.LaunchDescription([
        manager,
        spawner,
        robot_state_publisher,
        joint_state_publisher,
        register_on_exit
    ])
