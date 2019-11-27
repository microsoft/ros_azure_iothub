# Azure IoT Hub Relay for ROS (roscpp version)

# Prerequisites
To run this sample, you will need:
  * Azure IoT Hub service and at least one device registered under IoT Hub.
    * See more on [Quickstart: Send telemetry from a device to an IoT hub and read the telemetry from the hub with a back-end application](https://docs.microsoft.com/en-us/azure/iot-hub/quickstart-send-telemetry-c).
  * ROS installation.
    * For Windows environment, check [ROS on Windows](https://aka.ms/ros).
    * For Ubuntu Linux environment, check [Ubuntu install of ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu).

# How to Build (ROS on Windows)
Here is an example how to build it for ROS on Windows.
```
> :: source ROS environment
> c:\opt\ros\melodic\x64\setup.bat

> :: create catkin workspace folders
> mkdir catkin_ws\src
> pushd catkin_ws
> catkin_init_workspace src

> :: checkout required ROS package sources
> pushd src
> git clone https://github.com/ms-iot/abseil-cpp -b init_windows
> git clone https://github.com/ms-iot/ros_type_introspection -b init_windows
> git clone --recursive https://github.com/Microsoft/ros_azure_iothub
> popd

> :: install system dependencies
> rosdep update
> rosdep install --from-paths src --ignore-src -r -y

> :: build it and source install environment
> catkin_make install
> install\setup.bat
```

# How to Build (Ubuntu Linux Melodic install)
Here is an example how to build it for Ubuntu Linux Melodic install.
```
$ # source ROS environment
$ source /opt/ros/melodic/setup.bash

$ # create catkin workspace folders
$ mkdir catkin_ws\src -p
$ pushd catkin_ws
$ catkin_init_workspace src

$ # checkout required ROS package sources
$ pushd src
$ git clone --recursive https://github.com/Microsoft/ros_azure_iothub
$ popd

$ # install system dependencies
$ sudo rosdep update
$ sudo rosdep install --from-paths src --ignore-src -r -y

$ # install Azure IoT SDK as Debian
$ sudo add-apt-repository ppa:aziotsdklinux/ppa-azureiot
$ sudo apt-get update
$ sudo apt-get install -y azure-iot-sdk-c-dev

$ # build it and source install environment
$ chmod a+x src/ros_azure_iothub/dynamic_tutorials/cfg/tutorials.cfg
$ catkin_make install
$ source /install/setup.bash
```

# Deployment (IoT Hub for telemetry reporting)
Device twins are JSON documents that store device state information including metadata, configurations, and conditions. Azure IoT Hub maintains a device twin for each device that you connect to IoT Hub. And we are using `desired` properties as a channel to ask our ROS node what ROS topics to report.

Here is a JSON example to report `/rosout`:
```
{
    "deviceId": "devA",
    ...
    "properties": {
        "desired": {
            "ros_relays": {
                "1": "/rosout"
            },
            ...
        },
        ...
    }
}
```

And add the `ros_relays` block to the device twin which you are about to connect in the next step. Meanwhile, you can run the following Azure PowerShell cmdlet to wait for events from IoT Hub.

```
az iot hub monitor-events --hub-name <YourIoTHubName> --output table
```

# Deployment (IoT Hub for dynamic configuration)
[Dynamic Reconfiguration](http://wiki.ros.org/dynamic_reconfigure) provides a way to change the node parameters during runtime without restarting the node.
Similar as the telemetry reporting, we are using the device twin `desired` properties as a channel to ask our ROS node what dynamic parameters to reconfigure.

Here is a JSON example to reconfigure the parameters of `/dynamic_tutorials_node` with the new value:
```
{
    "deviceId": "devA",
    ...
    "properties": {
        "desired": {
            "ros_dynamic_configurations": {
                "0": {
                        "node": "/dynamic_tutorials_node", 
                        "param": "str_param",
                        "type":  "string",
                        "value": "HelloWorld!"
                },
                "1": {
                        "node": "/dynamic_tutorials_node",
                        "param": "int_param",
                        "type":  "int",
                        "value": "33"
                     },
                "2": {
                        "node": "/dynamic_tutorials_node",
                        "param": "double_param",
                        "type":  "double",
                        "value": "0.55"
                     },
                "3": {
                        "node": "/dynamic_tutorials_node",
                        "param": "bool_param",
                        "type":  "bool",
                        "value": "1"
                     }
            ...
        },
        ...
    }
}
```
Currently 4 types of parameters can be dynamically reconfigured, they are "string", "int", "double" and "bool".

# Deployment (Client side)
This node can be run using `roslaunch` (replacing the value for `connection_string` with the value given by Azure IoT Hub):
```
roslaunch roscpp_azure_iothub sample.launch connection_string:="HostName=sample.azure-devices.net;DeviceId=rosbot;SharedAccessKey=sampleKey"
```

This value can also be set in the ROS Parameter Server at `/roscpp_azure_iothub_node/connection_string`.

Now you can run some other ROS scenarios and see the `/rosout` being reported back to IoT Hub or the node parameters being dynamically reconfigured.

# Contributing

This project welcomes contributions and suggestions.  Most contributions require you to agree to a
Contributor License Agreement (CLA) declaring that you have the right to, and actually do, grant us
the rights to use your contribution. For details, visit https://cla.microsoft.com.

When you submit a pull request, a CLA-bot will automatically determine whether you need to provide
a CLA and decorate the PR appropriately (e.g., label, comment). Simply follow the instructions
provided by the bot. You will only need to do this once across all repos using our CLA.

This project has adopted the [Microsoft Open Source Code of Conduct](https://opensource.microsoft.com/codeofconduct/).
For more information see the [Code of Conduct FAQ](https://opensource.microsoft.com/codeofconduct/faq/) or
contact [opencode@microsoft.com](mailto:opencode@microsoft.com) with any additional questions or comments.
