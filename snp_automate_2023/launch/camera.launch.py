# Copyright (c) 2020, Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for
            param in parameters]


def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])


configurable_parameters = [
    {'name': 'camera_name',                  'default': 'camera', 'description': 'camera unique name'},
    {'name': 'serial_no',                    'default': "''", 'description': 'choose device by serial number'},
    {'name': 'usb_port_id',                  'default': "''", 'description': 'choose device by usb port id'},
    {'name': 'device_type',                  'default': "''", 'description': 'choose device by type'},
    {'name': 'config_file',                  'default': "''", 'description': 'yaml config file'},
    {'name': 'unite_imu_method',             'default': "0", 'description': '[0-None, 1-copy, 2-linear_interpolation]'},
    {'name': 'json_file_path',               'default': "''", 'description': 'allows advanced configuration'},
    {'name': 'log_level',                    'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
    {'name': 'output',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},
    {'name': 'depth_module.profile',         'default': '0,0,0', 'description': 'depth module profile'},
    {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
    {'name': 'rgb_camera.profile',           'default': '0,0,0', 'description': 'color image width'},
    {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
    {'name': 'enable_infra1',                'default': 'false', 'description': 'enable infra1 stream'},
    {'name': 'enable_infra2',                'default': 'false', 'description': 'enable infra2 stream'},
    {'name': 'infra_rgb',                    'default': 'false', 'description': 'enable infra2 stream'},
    {'name': 'tracking_module.profile',      'default': '0,0,0', 'description': 'fisheye width'},
    {'name': 'enable_fisheye1',              'default': 'true', 'description': 'enable fisheye1 stream'},
    {'name': 'enable_fisheye2',              'default': 'true', 'description': 'enable fisheye2 stream'},
    {'name': 'enable_confidence',            'default': 'true', 'description': 'enable depth stream'},
    {'name': 'gyro_fps',                     'default': '0', 'description': "''"},
    {'name': 'accel_fps',                    'default': '0', 'description': "''"},
    {'name': 'enable_gyro',                  'default': 'false', 'description': "''"},
    {'name': 'enable_accel',                 'default': 'false', 'description': "''"},
    {'name': 'enable_pose',                  'default': 'true', 'description': "''"},
    {'name': 'pose_fps',                     'default': '200', 'description': "''"},
    {'name': 'pointcloud.enable',            'default': 'false', 'description': ''},
    {'name': 'pointcloud.stream_filter',     'default': '2', 'description': 'texture stream for pointcloud'},
    {'name': 'pointcloud.stream_index_filter','default': '0', 'description': 'texture stream index for pointcloud'},
    {'name': 'enable_sync',                  'default': 'false', 'description': "''"},
    {'name': 'align_depth.enable',           'default': 'false', 'description': "''"},
    {'name': 'colorizer.enable',             'default': 'false', 'description': "''"},
    {'name': 'clip_distance',                'default': '-2.', 'description': "''"},
    {'name': 'linear_accel_cov',             'default': '0.01', 'description': "''"},
    {'name': 'initial_reset',                'default': 'false', 'description': "''"},
    {'name': 'allow_no_texture_points',      'default': 'false', 'description': "''"},
    {'name': 'ordered_pc',                   'default': 'false', 'description': ''},
    {'name': 'calib_odom_file',              'default': "''", 'description': "''"},
    {'name': 'topic_odom_in',                'default': "''", 'description': 'topic for T265 wheel odometry'},
    {'name': 'tf_publish_rate',              'default': '0.0', 'description': 'Rate of publishing static_tf'},
    {'name': 'diagnostics_period',           'default': '0.0', 'description': 'Rate of publishing diagnostics. 0=Disabled'},
    {'name': 'decimation_filter.enable',     'default': 'false', 'description': 'Rate of publishing static_tf'},
    {'name': 'rosbag_filename',              'default': "''", 'description': 'A realsense bagfile to run from as a device'},
    {'name': 'depth_module.exposure.1',      'default': '7500', 'description': 'Initial value for hdr_merge filter'},
    {'name': 'depth_module.gain.1',          'default': '16', 'description': 'Initial value for hdr_merge filter'},
    {'name': 'depth_module.exposure.2',      'default': '1', 'description': 'Initial value for hdr_merge filter'},
    {'name': 'depth_module.gain.2',          'default': '16', 'description': 'Initial value for hdr_merge filter'},
    {'name': 'wait_for_device_timeout',      'default': '-1.', 'description': 'Timeout for waiting for device to connect (Seconds)'},
    {'name': 'reconnect_timeout',            'default': '6.', 'description': 'Timeout(seconds) between consequtive reconnection attempts'},
    {'name': 'intra_process_comms',          'default': 'true', 'description': "enable intra-process communication"},
    {'name': 'container',                    'default': '', 'description': 'Name of an existing node container to load launched nodes into. If unset, a new container will be created.'},
]

composable_nodes = [
    ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='rectify_color_node',
        namespace=LaunchConfiguration("camera_name"),
        # Remap subscribers and publishers
        remappings=[
            ('image', 'color/image_raw'),
            ('image_rect', 'color/image_rect'),
            ('camera_info', 'color/camera_info'),
        ],
    ),
    ComposableNode(
        package='realsense2_camera',
        namespace=LaunchConfiguration("camera_name"),
        plugin='realsense2_camera::' + 'RealSenseNodeFactory',
        name="camera",
        parameters=[set_configurable_parameters(configurable_parameters)],
        extra_arguments=[{'use_intra_process_comms': LaunchConfiguration("intra_process_comms")}]
    ),
    ComposableNode(
        package='realsense2_camera',
        namespace=LaunchConfiguration("camera_name"),
        plugin='rs2_ros::tools::frame_latency::' + 'FrameLatencyNode',
        name='frame_latency',
        parameters=[set_configurable_parameters(configurable_parameters)],
        extra_arguments=[{'use_intra_process_comms': LaunchConfiguration("intra_process_comms")}]
    ),
]


def generate_launch_description():
    # If an existing container is not provided, start a container and load nodes into it
    image_processing_container = ComposableNodeContainer(
        condition=LaunchConfigurationEquals('container', ''),
        name='image_proc_container',
        namespace=LaunchConfiguration("camera_name"),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen',
        emulate_tty=True,  # needed for display of logs
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    # If an existing container name is provided, load composable nodes into it
    # This will block until a container with the provided name is available and nodes are loaded
    load_composable_nodes = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals('container', ''),
        composable_node_descriptions=composable_nodes,
        target_container=LaunchConfiguration('container'),
    )

    return LaunchDescription(declare_configurable_parameters(configurable_parameters) +
                             [image_processing_container,
                              load_composable_nodes,]
                             )
