// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#define _SILENCE_CXX17_OLD_ALLOCATOR_MEMBERS_DEPRECATION_WARNING
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <inttypes.h>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/exceptions.hpp" 

// Azure IoT Hub
#include <azure_macro_utils/macro_utils.h>

#include <azure_c_shared_utility/threadapi.h>
#include <azure_c_shared_utility/platform.h>
#include <azure_c_shared_utility/shared_util_options.h>
#include <iothub_device_client.h>
#include <iothub_client_options.h>
#include <iothub.h>
#include <iothub_message.h>
#include <parson.h>
#include <iothubtransportmqtt.h>

int g_interval = 10000;  // 10 sec send interval initially, currently not used
const std::string g_authentication_SAS = "SAS";
const std::string g_authentication_x509 = "x509";

std::shared_ptr<rclcpp::Node> nh;

class iotNode : public rclcpp::Node
{
    public:
        iotNode() : Node("ros_azure_iothub") {
            this->declare_parameter("connection_string", "HostName=sample.azure-devices.net;DeviceId=rosbot;SharedAccessKey=sampleKey");
            this->declare_parameter("authenticationType", "");
        }
    private:
};

struct ROS_Azure_IoT_Hub {
    IOTHUB_DEVICE_CLIENT_HANDLE deviceHandle;
};

static IOTHUBMESSAGE_DISPOSITION_RESULT receive_msg_callback(IOTHUB_MESSAGE_HANDLE message, void* user_context)
{
    (void)user_context;
    const char* messageId;
    const char* correlationId;
    // Message properties
    if ((messageId = IoTHubMessage_GetMessageId(message)) == NULL)
    {
        messageId = "<unavailable>";
    }

    if ((correlationId = IoTHubMessage_GetCorrelationId(message)) == NULL)
    {
        correlationId = "<unavailable>";
    }

    IOTHUBMESSAGE_CONTENT_TYPE content_type = IoTHubMessage_GetContentType(message);
    if (content_type == IOTHUBMESSAGE_BYTEARRAY)
    {
        const unsigned char* buff_msg;
        size_t buff_len;

        if (IoTHubMessage_GetByteArray(message, &buff_msg, &buff_len) != IOTHUB_MESSAGE_OK)
        {
            RCLCPP_ERROR(nh->get_logger(), "Failure retrieving byte array message");
        }
        else
        {
            RCLCPP_INFO(nh->get_logger(), "Received Binary message\r\n Message ID: %s\r\n Correlation ID: %s\r\n Data: <<<%.*s>>> & Size=%d", messageId, correlationId, (int)buff_len, buff_msg, (int)buff_len);
        }
    }
    else
    {
        const char* string_msg = IoTHubMessage_GetString(message);
        if (string_msg == NULL)
        {
            RCLCPP_ERROR(nh->get_logger(), "Failure retrieving byte array message");
        }
        else
        {
            RCLCPP_INFO(nh->get_logger(), "Received String Message\r\n Message ID: %s\r\n Correlation ID: %s\r\n Data: <<<%s>>>", messageId, correlationId, string_msg);
        }
    }
    return IOTHUBMESSAGE_ACCEPTED;
}

static void connection_status_callback(IOTHUB_CLIENT_CONNECTION_STATUS result, IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason, void* user_context)
{
    (void)reason;
    (void)user_context;

    // This sample DOES NOT take into consideration network outages.
    if (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED)
    {
        RCLCPP_INFO(nh->get_logger(), "The device client is connected to iothub");
    }
    else
    {
        RCLCPP_INFO(nh->get_logger(), "The device client has been disconnected");
    }
}


static void deviceTwinCallback(DEVICE_TWIN_UPDATE_STATE update_state, const unsigned char* payLoad, size_t size, void* userContextCallback)
{
    (void)update_state;
    (void)size;

    ROS_Azure_IoT_Hub* iotHub = (ROS_Azure_IoT_Hub*)userContextCallback;
    (void) iotHub; 

    JSON_Value* root_value = json_parse_string((const char*)payLoad);
    JSON_Object* root_object = json_value_get_object(root_value);
    JSON_Object* desired_object = json_object_dotget_object(root_object, "desired");
    JSON_Object* ros_object = (desired_object != NULL) ? desired_object : root_object;

    // Get dynamic reconfiguration settings
    JSON_Object* arrayObject = json_object_dotget_object(ros_object, "ros_dynamic_configurations");
    size_t objectCount = json_object_get_count(arrayObject);

    for (size_t i = 0; i < objectCount; i++)
    {
        JSON_Value* configure_value = json_object_get_value_at(arrayObject, i);
        JSON_Object* configure_object = json_value_get_object(configure_value);

        const char *node = NULL, *param = NULL, *type = NULL, *value = NULL;
        JSON_Value* node_value = json_object_get_value(configure_object, "node");
        if (node_value != NULL)
        {
            node = json_value_get_string(node_value);
        }

        JSON_Value* param_value = json_object_get_value(configure_object, "param");
        if (param_value != NULL)
        {
            param = json_value_get_string(param_value);
        }

        JSON_Value* type_value = json_object_get_value(configure_object, "type");
        if (type_value != NULL)
        {
            type = json_value_get_string(type_value);
        }

        JSON_Value* val_value = json_object_get_value(configure_object, "value");
        if (val_value != NULL)
        {
            value = json_value_get_string(val_value);
        }

        if (node != NULL && param != NULL && type != NULL && value != NULL) 
        {
            RCLCPP_INFO(nh->get_logger(), "Trying to send dynamic configuration command - node:%s, parameter:%s, data type:%s, value:%s", node, param, type, value);

            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(nh, node); //node

            while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(nh->get_logger(), "Interrupted while waiting for the parameter service. Exiting.");
                    return;
                }
                RCLCPP_INFO(nh->get_logger(), "Parameter service not available, waiting again...");
            }

            RCLCPP_INFO(nh->get_logger(), "Parameter service is available");

        
        try
        {
            rclcpp::Parameter updated_param = rclcpp::Parameter(param, value);
            rcl_interfaces::msg::SetParametersResult result;
            result = parameters_client->set_parameters_atomically({updated_param});

            if (result.successful == true)
            {
                RCLCPP_INFO(nh->get_logger(), "Successfully set parameter");
            }
            else
            {
                RCLCPP_INFO(nh->get_logger(), "Could not set parameter");
            }
        }
        catch (const std::exception& e) // TODO: Exception thrown: Node has already been added to an executor.
        {
            RCLCPP_ERROR(nh->get_logger(), "Exception thrown while setting parameter: %s", e.what());
        }
        
        } 
    }

    // Free resources
    json_value_free(root_value);
}


static bool ReadKeyFromFile(const std::string fileName, std::string &key)
{ 
    key = "";
    if (fileName.empty())
    {
        RCLCPP_ERROR(nh->get_logger(), "File name empty.");
        return false;
    }

    std::ifstream f(fileName); //taking file as inputstream
    if(f)
    {
        try
        {
            std::ostringstream ss;
            ss << f.rdbuf(); // reading data
            key = ss.str();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(nh->get_logger(), "Exception thrown during file read: %s", e.what());
        }
    }
    else
    {
        RCLCPP_ERROR(nh->get_logger(), "Could not read from file.");
        return false;
    }

    RCLCPP_INFO(nh->get_logger(), "File Read: %s", fileName.c_str());
    return true;
}

static bool InitializeX509Certificate(IOTHUB_DEVICE_CLIENT_HANDLE deviceHandle)
{
    std::string x509certificateFile;
    std::string x509certificate;

    nh->get_parameter("public_key_file", x509certificateFile);
    RCLCPP_INFO(nh->get_logger(), "public_key_file: %s", x509certificateFile.c_str());
    if (!ReadKeyFromFile(x509certificateFile, x509certificate))
    {
        RCLCPP_ERROR(nh->get_logger(), "Could not read x509 certificate/public key file, aborting.");
        return false;
    }

    std::string x509privatekeyFile;
    std::string x509privatekey;
    nh->get_parameter("private_key_file", x509privatekeyFile);
    if (!ReadKeyFromFile(x509privatekeyFile, x509privatekey))
    {
        RCLCPP_ERROR(nh->get_logger(), "Could not read x509 private key file, aborting.");
        return false;
    }

    // Set the X509 certificates in the SDK
    IOTHUB_CLIENT_RESULT status;
    status = IoTHubDeviceClient_SetOption(deviceHandle, OPTION_X509_CERT, x509certificate.c_str());
    if (status != IOTHUB_CLIENT_OK)
    {
        RCLCPP_ERROR(nh->get_logger(), "Failed to set option for x509 certificate: %s", IOTHUB_CLIENT_RESULTStrings(status));
        return false;
    }

    status = IoTHubDeviceClient_SetOption(deviceHandle, OPTION_X509_PRIVATE_KEY, x509privatekey.c_str());
    if (status != IOTHUB_CLIENT_OK)
    {
        RCLCPP_ERROR(nh->get_logger(), "Failed to set option for x509 private key: %s", IOTHUB_CLIENT_RESULTStrings(status));
        return false;
    }

    RCLCPP_INFO(nh->get_logger(), "x509 certificate Succeeded.");

    return true;
}

static bool InitializeAzureIoTHub(ROS_Azure_IoT_Hub* iotHub)
{
    IOTHUB_CLIENT_TRANSPORT_PROVIDER protocol = MQTT_Protocol;

    (void)IoTHub_Init();
    
    RCLCPP_INFO(nh->get_logger(), "Creating IoTHub Device handle");

    std::string connectionString;
    nh->get_parameter("connection_string", connectionString);
    RCLCPP_INFO(nh->get_logger(), "connection_string: %s", connectionString.c_str());

    std::string authenticationType;
    nh->get_parameter("authentication_type", authenticationType);
    RCLCPP_INFO(nh->get_logger(), "authentication_type: %s", authenticationType.c_str());

    // Create the iothub handle here
    iotHub->deviceHandle = IoTHubDeviceClient_CreateFromConnectionString(connectionString.c_str(), protocol);
    if (iotHub->deviceHandle == NULL)
    {
        RCLCPP_ERROR(nh->get_logger(), "Failure createing Iothub device.  Hint: Check you connection string.");
        return false;
    }

    // Check if a x509 certificate should be used for authentication. If not,
    // a sharedAccessKey is expected in the connection string.
    if (authenticationType == g_authentication_SAS || authenticationType == "")
    {
        RCLCPP_INFO(nh->get_logger(), "Using Shared Access Signatures authentication.");
    }
    else if (authenticationType == g_authentication_x509)
    {
        RCLCPP_INFO(nh->get_logger(), "Using x.509 Certificate authentication.");
        if(!InitializeX509Certificate(iotHub->deviceHandle))
        {
            RCLCPP_ERROR(nh->get_logger(), "Failed to initialize x509 certificate.");
            return false;
        }
    }
    else
    {
        RCLCPP_ERROR(nh->get_logger(), "Invalid authentication type: %s", authenticationType.c_str());
        return false;
    }

    // Setting message callback to get C2D messages
    (void)IoTHubDeviceClient_SetMessageCallback(iotHub->deviceHandle, receive_msg_callback, NULL);
    // Setting connection status callback to get indication of connection to iothub
    (void)IoTHubDeviceClient_SetConnectionStatusCallback(iotHub->deviceHandle, connection_status_callback, NULL);
    (void)IoTHubDeviceClient_SetDeviceTwinCallback(iotHub->deviceHandle, deviceTwinCallback, iotHub);  //Device Twin callback requires context to send message and device Handle

    return true;
}

static void DeinitializeAzureIoTHub(ROS_Azure_IoT_Hub* iotHub)
{
    RCLCPP_INFO(nh->get_logger(), "Deinitializing IoTHub Device client");
    IoTHubDeviceClient_Destroy(iotHub->deviceHandle);
    IoTHub_Deinit();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    ROS_Azure_IoT_Hub iotHub;

    nh = std::make_shared<iotNode>();

    if (!InitializeAzureIoTHub(&iotHub))
    {
        return -1;
    }

    rclcpp::spin(nh);
    rclcpp::shutdown();

    DeinitializeAzureIoTHub(&iotHub);
    return 0;
}
