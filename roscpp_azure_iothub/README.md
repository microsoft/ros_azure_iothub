# Azure IoT Hub Relay for ROS

# Prerequisites
To use this ROS node on your Robot, you will need:
  * Azure IoT Hub service and at least one device registered under IoT Hub.
    * See more on [Quickstart: Send telemetry from a device to an IoT hub and read the telemetry from the hub with a back-end application](https://docs.microsoft.com/en-us/azure/iot-hub/quickstart-send-telemetry-c).
  * ROS installation.
    * For Windows environment, check [ROS on Windows](https://aka.ms/ros).
    * For Ubuntu Linux environment, check [Ubuntu install of ROS2 Foxy](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/).

# How to Build (ROS on Windows)
Here is an example how to build it for ROS on Windows.
```Batchfile
:: source ROS environment
c:\opt\ros\foxy\x64\setup.bat

:: create workspace folders
mkdir ws\src
pushd ws

:: checkout required ROS package sources
pushd src
git clone --recursive https://github.com/Microsoft/ros_azure_iothub
popd

:: install system dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

:: build it and source install environment
colcon build --packages-select roscpp_azure_iothub
install\local_setup.bat
```

# How to Build (Ubuntu Linux Melodic install)
Here is an example how to build it for Ubuntu Linux Melodic install.

``` bash
# source ROS environment
source /opt/ros/foxy/setup.bash

# create workspace folders
mkdir ws/src -p

# checkout required ROS package sources
pushd src
git clone --recursive https://github.com/Microsoft/ros_azure_iothub
popd

# install Azure IoT SDK as Debian
sudo add-apt-repository ppa:aziotsdklinux/ppa-azureiot

# update apt to refresh package registries
sudo apt update
sudo apt install -y azure-iot-sdk-c-dev

# install system dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# build it and source install environment
colcon build 
. install/setup.bash
```

# Deployment (IoT Hub for dynamic configuration)
[Dynamic Reconfiguration](https://design.ros2.org/articles/ros_parameters.html), a concept in ROS1, provides a way to change the node parameters during runtime without restarting the node. In ROS2 there is not an explicit tool called dynamic reconfiguration, however, 
parameters are hosted in nodes and thus can be dynamically reconfigured during runtime using the [Parameters Client](http://docs.ros2.org/bouncy/api/rclcpp/classrclcpp_1_1_sync_parameters_client.html) interface. 
We are using the device twin `desired` properties as a channel to ask our ROS node what node parameters to reconfigure.

Here is a JSON example to reconfigure the parameters of `/dynamic_tutorials_node` with the new value:

```json
{
    "deviceId": "devA",
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
        }
    }
}
```

Currently 4 types of parameters can be dynamically reconfigured, they are "string", "int", "double" and "bool".

# Deployment (Client side)

Set the connection_string parameter by updating the `connection_string` in the sample.launch.py file with the value given by Azure IoT Hub The format should be `"HostName=sample.azure-devices.net;DeviceId=rosbot;SharedAccessKey=sampleKey"`. 

This node can be run using `ros2 launch`:

``` bash
ros2 launch roscpp_azure_iothub sample.launch.py 
```

Now you can run some other ROS scenarios and see the node parameters being dynamically reconfigured.

# X.509 Certificate Authentication

By default, the ROS node will use SAS (Shared Access Signatures) to communicate with IoT Hub. Follow the instructions below to use X.509 certificates instead.

First, [set up X.509 security in your Azure IoT hub](https://docs.microsoft.com/en-us/azure/iot-hub/iot-hub-security-x509-get-started#authenticate-your-x509-device-with-the-x509-certificates). Complete the steps until you reach "Authenticate your X.509 device with the X.509 certificates". Instead, the ROS node will be used to connect using the X.509 certificates.

After successfully creating, generating or puchasing the X.509 certificates, you should have a public key/certificate and a private key. If the keys were created for test purposes using these instructions, they  should be stored in two files:`myDevice-private.pem` and `myDevice-public.pem`

Set the DEVICE_PRIVATE_KEY and DEVICE_PUBLIC_KEY environment variables to the full file name for the X.509 keys. For example, on Windows run (replacing 'myCertPath/myDevice' with the path and file name):
```Batchfile
set DEVICE_PRIVATE_KEY="C:/myCertPath/myDevice-private.pem"
set DEVICE_PUBLIC_KEY="C:/myCertPath/myDevice-public.pem"
```

Set the connection_string parameter by updating the `connection_string` in the sample_x509.launch.py file with the value given by Azure IoT Hub.The format should be `"HostName=sample.azure-devices.net;DeviceId=rosbot;SharedAccessKey=sampleKey"`. Similarly, update the `private_key_file`, and `public_key_file` parameters in the same launch file. 

To deploy, run the following.
```
::Launch the node using the X.509 sample launch file
ros2 launch roscpp_azure_iothub sample_x509.launch.py
```

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
