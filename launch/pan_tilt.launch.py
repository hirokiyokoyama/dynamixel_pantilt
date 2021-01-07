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
camera_calibration = os.path.join(
    share_dir, 'config', 'camera.yaml')

def generate_launch_description():
    manager = launch_ros.actions.Node(
        package = 'dynamixel_controllers',
        node_executable = 'controller_manager.py',
        output = 'screen',
        #node_name = 'dynamixel_manager',
        parameters = [manager_params]
    )

    spawner = launch_ros.actions.Node(
        package = 'dynamixel_controllers',
        node_executable = 'controller_spawner.py',
        output = 'screen',
        node_name = 'dynamixel_controller_spawner',
        parameters = [spawner_params],
        arguments = [
            '--manager=dynamixel_controller_manager',
            '--port=pantilt',
            'pan_controller',
            'tilt_controller'
        ]
    )

    joint_state_publisher = launch_ros.actions.Node(
        package = 'dynamixel_pantilt',
        node_executable = 'dynamixel_joint_state_publisher.py',
        output = 'screen',
        node_name = 'joint_state_publisher',
        parameters = [{
            'dynamixel_controllers': ['pan_controller', 'tilt_controller'] 
        }]
    )

    import tempfile
    import xacro
    with tempfile.NamedTemporaryFile(mode='w', delete=False) as urdf_file:
        doc = xacro.process_file(pantilt_xacro)
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

    camera = launch_ros.actions.Node(
        package = 'usb_camera_driver',
        node_executable = 'usb_camera_driver_node',
        output = 'screen',
        node_name = 'camera',
        parameters = [{
            'image_width': 640,
            'image_height': 480,
            'frame_id': 'camera_color_optical_frame',
            'camera_calibration_file': 'file://'+camera_calibration
        }]
    )

    camera_compress = launch_ros.actions.Node(
        package = 'image_transport',
        node_executable = 'republish',
        output = 'screen',
        node_name = 'camera_compress',
        remappings = [
            ('/in', '/image'),
            ('/out/compressed', '/image_compressed')
        ],
        arguments = ['raw', 'compressed']
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
        camera,
        #camera_compress,
        register_on_exit
    ])
