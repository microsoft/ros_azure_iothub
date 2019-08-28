#include <stdlib.h>
#include <stdio.h>

#include <azure_c_shared_utility/platform.h>
#include <iothub_service_client_auth.h>
#include <iothub_devicetwin.h>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace std;
static string connectionString, deviceId;

// Update device twin desired properties by invoking the Azure IOT Hub Service SDK
bool SetDeviceTwin(const char *update_device_twin_json)
{
    bool result = false;

    (void)platform_init();

    IOTHUB_SERVICE_CLIENT_AUTH_HANDLE iotHubServiceClientHandle = IoTHubServiceClientAuth_CreateFromConnectionString(connectionString.c_str());
    if (iotHubServiceClientHandle != NULL)
    {
        IOTHUB_SERVICE_CLIENT_DEVICE_TWIN_HANDLE serviceClientDeviceTwinHandle = IoTHubDeviceTwin_Create(iotHubServiceClientHandle);
        if (serviceClientDeviceTwinHandle != NULL)
        {
            char* deviceTwinJson = NULL;
            if ((deviceTwinJson = IoTHubDeviceTwin_GetTwin(serviceClientDeviceTwinHandle, deviceId.c_str())) != NULL)
            {
                char* updatedDeviceTwinJson = NULL;

                if ((updatedDeviceTwinJson = IoTHubDeviceTwin_UpdateTwin(serviceClientDeviceTwinHandle, deviceId.c_str(), update_device_twin_json)) != NULL)
                {
                    free(updatedDeviceTwinJson);
                    result = true;
                }
                free(deviceTwinJson);
            }
            IoTHubDeviceTwin_Destroy(serviceClientDeviceTwinHandle);
        }
        IoTHubServiceClientAuth_Destroy(iotHubServiceClientHandle);
    }

    platform_deinit();

    return result;
}

TEST(IOTHubTester, ReconfigurationTest)
{
    // Update device twin with the new value for str_param
    string expectedStr = "ROS_AZURE_IOT_HUB_RECONFIGURATION_TEST_" + std::to_string(ros::Time::now().toSec());
    string updateJson = "{\"properties\":{\"desired\":{\"ros_dynamic_configurations\":{\"0\":{ \"node\":\"/dynamic_tutorials_node\",\"param\": \"str_param\",\"value\":\"" + expectedStr + "\",\"type\":\"string\"}}}}}";
    EXPECT_TRUE(SetDeviceTwin(updateJson.c_str()));

    ros::NodeHandle nh;
    string actualStr;
    int count = 0;
    ros::Rate r(1);
    while (ros::ok() && count < 10)
    {
        // Get the dynamic configuration parameter value
        if (nh.getParam("/dynamic_tutorials_node/str_param", actualStr))
        {
            if (expectedStr == actualStr)
            {
                break;
            }
        }

        count++;
        ros::spinOnce();
        r.sleep();
    }

    EXPECT_EQ(expectedStr, actualStr);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "azure_iot_hub_test");

    ros::NodeHandle nh("~");
    nh.getParam("iot_hub_connection_string", connectionString);
    nh.getParam("device_name", deviceId);

    // Wait 5 seconds to make sure roscpp_azure_iothub_node initialization completes.
    ros::Duration(5).sleep();
    return RUN_ALL_TESTS();
}