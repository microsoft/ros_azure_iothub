# Azure IoT Hub Relay for ROS

# Deployment
This node can be run using `roslaunch` (replacing the value for `connection_string` with the value given by Azure IoT Hub):
```
roslaunch ros_azure_iothub sample.launch connection_string:="HostName=sample.azure-devices.net;DeviceId=rosbot;SharedAccessKey=sampleKey"
```

This value can also be set in the ROS Parameter Server at `/ros_azure_iothub/connection_string`.

## Deployment Considerations (Updated 12/04/2017)
* The [Microsoft Azure IoT SDK for Python](https://github.com/Azure/azure-iot-sdk-python) is compiled
using libboost1.54, which is not easily installable on all Linux distributions. If installing libboost1.54 is not an option for your
distribution, please clone and compile the Azure IoT SDK for Python from source. Once complete, place the resulting iothub_client.so
in the same working directory as the ROS node.

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
