# Azure IoT Hub Relay for ROS

# Deployment
This node can be run using `roslaunch` (replacing the value for `connection_string` with the value given by Azure IoT Hub):
```
roslaunch ros_azure_iothub sample.launch connection_string:="HostName=sample.azure-devices.net;DeviceId=rosbot;SharedAccessKey=sampleKey"
```

This value can also be set in the ROS Parameter Server at `/ros_azure_iothub/connection_string`.

## Deployment Considerations (Updated 12/04/2017)
* The [Microsoft Azure IoT SDK for Python](https://github.com/Azure/azure-iot-sdk-python) is compiled
using libboost1.54, which is not easily installable on all Linux distributions. If this is the case, please clone and
compile this library from source. Once complete, place the resulting iothub_client.so in the same working directory as the ROS node.

* The [Microsoft Azure IoT SDK for Python](https://github.com/Azure/azure-iot-sdk-python) installs a library (iothub_client.so)
that cannot be resolved using a normal Python import strategy. In the node, the script will attempt to locate the iothub_client.so
file in the dist-packages folder for Python 2.7 and add the package folder to the path for the duration of execution. If there
are still fatal errors when running this package, please place the iothub_client.so file in the working directory of the node,
or modify the node script to specify the correct location to find the iothub_client.so file.

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
