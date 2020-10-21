# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roscpp_azure_iothub',
            namespace='roscpp_azure_iothub_node',
            executable='roscpp_azure_iothub_node',
            name='roscpp_azure_iothub_node',
            parameters=[
                {"connection_string": "HostName=sample.azure-devices.net;DeviceId=rosbot;SharedAccessKey=sampleKey", 
                    "authenticationType": "x509", 
                    "private_key_file": "",
                    "public_key_file": ""} # Update your parameters here. 
            ]
        )
    ])
