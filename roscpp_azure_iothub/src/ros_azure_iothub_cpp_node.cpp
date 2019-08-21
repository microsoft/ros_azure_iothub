// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <stdlib.h>
#include <inttypes.h>
#include <stdio.h>

// Azure IoT Hub
#ifdef _WIN32
#include <azure_c_shared_utility/macro_utils.h>
#else
#include <azure_macro_utils/macro_utils.h>
#endif

#include <azure_c_shared_utility/threadapi.h>
#include <azure_c_shared_utility/platform.h>
#include <iothub_device_client.h>
#include <iothub_client_options.h>
#include <iothub.h>
#include <iothub_message.h>
#include <parson.h>
#include <iothubtransportmqtt.h>

// ROS Introspection
#include <ros_type_introspection/ros_introspection.hpp>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

using namespace RosIntrospection;

// Dynamic Reconfigures
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

using namespace dynamic_reconfigure;

int g_interval = 10000;  // 10 sec send interval initially, currently not used
static size_t g_message_count_send_confirmations = 0;
static bool g_is_startup = true;

struct ROS_Azure_IoT_Hub {
    IOTHUB_DEVICE_CLIENT_HANDLE deviceHandle;
    Parser parser;
    ros::NodeHandle nh;
    std::vector<ros::Subscriber> subscribers;
	std::vector<std::string> topicsToSubscribe;
};

static IOTHUBMESSAGE_DISPOSITION_RESULT receive_msg_callback(IOTHUB_MESSAGE_HANDLE message, void* user_context)
{
    (void)user_context;
    const char* messageId;
    const char* correlationId;
	
	ROS_INFO("receive_msg_callback\r\n");

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
            ROS_ERROR("Failure retrieving byte array message\r\n");
        }
        else
        {
            ROS_INFO("Received Binary message\r\n Message ID: %s\r\n Correlation ID: %s\r\n Data: <<<%.*s>>> & Size=%d\r\n", messageId, correlationId, (int)buff_len, buff_msg, (int)buff_len);
        }
    }
    else
    {
        const char* string_msg = IoTHubMessage_GetString(message);
        if (string_msg == NULL)
        {
            ROS_ERROR("Failure retrieving byte array message\r\n");
        }
        else
        {
            ROS_INFO("Received String Message\r\n Message ID: %s\r\n Correlation ID: %s\r\n Data: <<<%s>>>\r\n", messageId, correlationId, string_msg);
        }
    }
    return IOTHUBMESSAGE_ACCEPTED;
}

static int device_method_callback(const char* method_name, const unsigned char* payload, size_t size, unsigned char** response, size_t* resp_size, void* userContextCallback)
{
    const char* SetTelemetryIntervalMethod = "SetTelemetryInterval";
    const char* device_id = (const char*)userContextCallback;
    char* end = NULL;
    int newInterval;

    int status = 501;
    const char* RESPONSE_STRING = "{ \"Response\": \"Unknown method requested.\" }";

    ROS_INFO("\r\nDevice Method called for device %s\r\n", device_id);
    ROS_INFO("Device Method name:    %s\r\n", method_name);
    ROS_INFO("Device Method payload: %.*s\r\n", (int)size, (const char*)payload);

    if (strcmp(method_name, SetTelemetryIntervalMethod) == 0)
    {
        if (payload)
        {
            newInterval = (int)strtol((char*)payload, &end, 10);

            // Interval must be greater than zero.
            if (newInterval > 0)
            {
                // Expect sec and covert to ms
                g_interval = 1000 * (int)strtol((char*)payload, &end, 10);
                status = 200;
                RESPONSE_STRING = "{ \"Response\": \"Telemetry reporting interval updated.\" }";
            }
            else
            {
                status = 500;
                RESPONSE_STRING = "{ \"Response\": \"Invalid telemetry reporting interval.\" }";
            }
        }
    }

    ROS_INFO("\r\nResponse status: %d\r\n", status);
    ROS_INFO("Response payload: %s\r\n", RESPONSE_STRING);

    *resp_size = strlen(RESPONSE_STRING);
    *response = reinterpret_cast<unsigned char*>(malloc(*resp_size));
    if (*response)
    {
        memcpy(*response, RESPONSE_STRING, *resp_size);
    }
    else
    {
        status = -1;
    }
    return status;
}

static void connection_status_callback(IOTHUB_CLIENT_CONNECTION_STATUS result, IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason, void* user_context)
{
    (void)reason;
    (void)user_context;
	ROS_INFO("connection_status_callback\r\n");
    // This sample DOES NOT take into consideration network outages.
    if (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED)
    {
        ROS_INFO("The device client is connected to iothub\r\n");
    }
    else
    {
        ROS_INFO("The device client has been disconnected\r\n");
    }
}

static void send_confirm_callback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* userContextCallback)
{
    (void)userContextCallback;
    // When a message is sent this callback will get envoked
    g_message_count_send_confirmations++;
    #ifdef _WIN32
    ROS_INFO("Confirmation callback received for message %lu with result %s\r\n", (unsigned long)g_message_count_send_confirmations, ENUM_TO_STRING(IOTHUB_CLIENT_CONFIRMATION_RESULT, result));
    #else
    ROS_INFO("Confirmation callback received for message %lu with result %s\r\n", (unsigned long)g_message_count_send_confirmations, MU_ENUM_TO_STRING(IOTHUB_CLIENT_CONFIRMATION_RESULT, result));
    #endif
}

static bool IsTopicAvailableForSubscribe(const char* topicName)
{
    std::string topic_name = topicName;
    
    ros::master::V_TopicInfo advertized_topics;
    ros::master::getTopics(advertized_topics);

    bool found = false;
    for (const auto& topic_info: advertized_topics)
    {
        if( topic_info.name == topic_name)
        {
            found = true;
            break;
        }
    }
    if (!found )
    {
        ROS_WARN("This topic has not been published yet: %s\r\n", topic_name.c_str());
        return false;
    }
    return true;
}


void sendMsgToAzureIoTHub(const char* msg, IOTHUB_DEVICE_CLIENT_HANDLE deviceHandle)
{
    static int messagecount;
    
    // Construct the iothub message from a string or a byte array
    IOTHUB_MESSAGE_HANDLE message_handle = IoTHubMessage_CreateFromString(msg);

    // Set Message property
    (void)IoTHubMessage_SetMessageId(message_handle, "MSG_ID");
    (void)IoTHubMessage_SetCorrelationId(message_handle, "CORE_ID");
    (void)IoTHubMessage_SetContentTypeSystemProperty(message_handle, "application%2fjson");
    (void)IoTHubMessage_SetContentEncodingSystemProperty(message_handle, "utf-8");

    // Add custom properties to message 
    //(void)IoTHubMessage_SetProperty(message_handle, "property_key", "property_value");

    ROS_INFO("\r\nSending message %d to IoTHub\r\nMessage: %s\r\n", (int)(messagecount + 1), msg);
    IoTHubDeviceClient_SendEventAsync(deviceHandle, message_handle, send_confirm_callback, NULL);

    // The message is copied to the sdk so the we can destroy it
    IoTHubMessage_Destroy(message_handle);
    messagecount++;
}

void topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg,
                   const std::string &topic_name,
                   RosIntrospection::Parser& parser,
                   IOTHUB_DEVICE_CLIENT_HANDLE deviceHandle)
{
    ROS_INFO("topicCallback\r\n");
    const std::string&  datatype   =  msg->getDataType();
    const std::string&  definition =  msg->getMessageDefinition();

    // Don't worry if you do this more than once: already registered message are not overwritten.
    parser.registerMessageDefinition( topic_name,
                                      RosIntrospection::ROSType(datatype),
                                      definition );

    // Reuse these opbects to improve efficiency ("static" makes them persistent)
    static std::vector<uint8_t> buffer;
    static std::map<std::string,FlatMessage>   flat_containers;
    static std::map<std::string,RenamedValues> renamed_vectors;

    FlatMessage&   flat_container = flat_containers[topic_name];
    RenamedValues& renamed_values = renamed_vectors[topic_name];

    // Copy raw memory into the buffer
    buffer.resize( msg->size() );
    ros::serialization::OStream stream(buffer.data(), buffer.size());
    msg->write(stream);

    // Deserialize and rename the vectors
    parser.deserializeIntoFlatContainer( topic_name, absl::Span<uint8_t>(buffer), &flat_container, 100);
    parser.applyNameTransform( topic_name, flat_container, &renamed_values );

    // Print the content of the message
    ROS_INFO("--------- %s ----------\r\n", topic_name.c_str() );
    sendMsgToAzureIoTHub(topic_name.c_str(), deviceHandle);
    for (auto it: renamed_values)
    {
        const std::string& key = it.first;
        const Variant& value   = it.second;
        char buffer [256] = {0};
        snprintf(buffer, sizeof(buffer), " %s = %f", key.c_str(), value.convert<double>());
        ROS_INFO("%s\r\n",buffer);
        sendMsgToAzureIoTHub(buffer, deviceHandle);
    }
    for (auto it: flat_container.name)
    {
        const std::string& key    = it.first.toStdString();
        const std::string& value  = it.second;
        char buffer [256] = {0};
        snprintf(buffer, sizeof(buffer), " %s = %s", key.c_str(), value.c_str());
        ROS_INFO("%s\r\n",buffer);
        sendMsgToAzureIoTHub(buffer, deviceHandle);
    }
}

static void subscribeTopic(const char* topicName, ROS_Azure_IoT_Hub* iotHub)
{
    boost::function<void(const topic_tools::ShapeShifter::ConstPtr&) > callback;
    callback = [iotHub, topicName](const topic_tools::ShapeShifter::ConstPtr& msg) -> void
    {
        topicCallback(msg, topicName, iotHub->parser, iotHub->deviceHandle) ;
    };
    iotHub->subscribers.push_back( iotHub->nh.subscribe(topicName, 10, callback) );
}

// Current code does not keep track of topics already subscribed
static void deviceTwinCallback(DEVICE_TWIN_UPDATE_STATE update_state, const unsigned char* payLoad, size_t size, void* userContextCallback)
{
    (void)update_state;
    (void)size;

    ROS_Azure_IoT_Hub* iotHub = (ROS_Azure_IoT_Hub*)userContextCallback;

    JSON_Value* root_value = json_parse_string((const char*)payLoad);
    JSON_Object* root_object = json_value_get_object(root_value);
    JSON_Object* desired_object = json_object_dotget_object(root_object, "desired");
    JSON_Object* ros_object = (desired_object != NULL) ? desired_object : root_object;

    // Get topics for subscription
    JSON_Object* arrayObject = json_object_dotget_object(ros_object, "ros_relays");
    size_t objectCount = json_object_get_count(arrayObject);

    for (size_t i = 0; i < objectCount; i++)
    {
        ROS_INFO("  %s\r\n", json_object_get_name(arrayObject, i));
        JSON_Value* value = json_object_get_value_at(arrayObject, i);
        const char* topicToSubscribe = json_value_get_string(value);
        ROS_INFO("  %s\r\n", json_value_get_string(value));

        // Only subscribe the topic that is avaiable but not subscribed before
        if (topicToSubscribe != NULL && IsTopicAvailableForSubscribe(topicToSubscribe) && std::find(iotHub->topicsToSubscribe.begin(), iotHub->topicsToSubscribe.end(), topicToSubscribe) == iotHub->topicsToSubscribe.end())
        {
            ROS_INFO("Subscribe topic:  %s\r\n", topicToSubscribe);
            iotHub->topicsToSubscribe.push_back(topicToSubscribe);
            subscribeTopic(topicToSubscribe, iotHub);
        }
    }

    if (!g_is_startup)
    {
        // Note: We ignore the existing ros_dynamic_configurations settings during ros_azure_iothub_cpp_node starts up.
        // Reason: Suppose the other nodes are working with the expected dynamic configurations,
        // and no need to update their configurations with the old values.

        // Get dynamic reconfiguration settings
        arrayObject = json_object_dotget_object(ros_object, "ros_dynamic_configurations");
        objectCount = json_object_get_count(arrayObject);

        for (size_t i = 0; i < objectCount; i++)
        {
            JSON_Value* congifure_value = json_object_get_value_at(arrayObject, i);
            JSON_Object* congifure_object = json_value_get_object(congifure_value);

            const char *node = NULL, *param = NULL, *type = NULL, *value = NULL;
            JSON_Value* node_value = json_object_get_value(congifure_object, "node");
            if (node_value != NULL)
            {
                node = json_value_get_string(node_value);
            }

            JSON_Value* param_value = json_object_get_value(congifure_object, "param");
            if (param_value != NULL)
            {
                param = json_value_get_string(param_value);
            }

            JSON_Value* type_value = json_object_get_value(congifure_object, "type");
            if (type_value != NULL)
            {
                type = json_value_get_string(type_value);
            }

            JSON_Value* val_value = json_object_get_value(congifure_object, "value");
            if (val_value != NULL)
            {
                value = json_value_get_string(val_value);
            }

            if (node != NULL && param != NULL && type != NULL && value != NULL)
            {
                ROS_INFO("Trying to send dynamic configuration command - node:%s, parameter:%s, data type:%s, value:%s\r\n", node, param, type, value);

                ReconfigureRequest srv_req;
                ReconfigureResponse srv_resp;
                Config conf;

                if (strcmp(type, "string") == 0)
                {
                    StrParameter string_param;
                    string_param.name = param;
                    string_param.value = value;
                    conf.strs.push_back(string_param);
                }
                else if (strcmp(type, "int") == 0)
                {
                    char* end = NULL;
                    IntParameter int_param;
                    int_param.name = param;
                    int_param.value = (int)strtol(value, &end, 10);
                    conf.ints.push_back(int_param);
                }
                else if (strcmp(type, "double") == 0)
                {
                    char* end = NULL;
                    DoubleParameter double_param;
                    double_param.name = param;
                    double_param.value = (double)strtod(value, &end);
                    conf.doubles.push_back(double_param);
                }
                else if (strcmp(type, "bool") == 0)
                {
                    try
                    {
                        BoolParameter bool_param;
                        bool_param.name = param;
                        bool_param.value = boost::lexical_cast<bool>(value);
                        conf.bools.push_back(bool_param);
                    }
                    catch (const boost::bad_lexical_cast &e)
                    {
                        (void)e;
                        ROS_ERROR("Failure converting %s to bool type\r\n", value);
                    }
                }

                srv_req.config = conf;
                char node_set_param[256] = {0};
                snprintf(node_set_param, sizeof(node_set_param), "%s/set_parameters", node);
                if (ros::service::call(node_set_param, srv_req, srv_resp))
                {
                    ROS_INFO("Succeed.\r\n");
                }
                else
                {
                    ROS_ERROR("Failure executing the dynamic configuration command.\r\n");
                }
            }
        }
    }
    else
    {
        g_is_startup = false;
    }

    // Free resources
    json_value_free(root_value);
}


static bool InitializeAzureIoTHub(ROS_Azure_IoT_Hub* iotHub)
{
    IOTHUB_CLIENT_TRANSPORT_PROVIDER protocol = MQTT_Protocol;

    (void)IoTHub_Init();
    ROS_INFO("Creating IoTHub Device handle\r\n");

    ros::NodeHandle nh("~");
    std::string connectionString;
    nh.getParam("connection_string", connectionString);
    ROS_INFO("connection_string: %s\r\n", connectionString.c_str());

    // Create the iothub handle here
    iotHub->deviceHandle = IoTHubDeviceClient_CreateFromConnectionString(connectionString.c_str(), protocol);
    if (iotHub->deviceHandle == NULL)
    {
        ROS_ERROR("Failure createing Iothub device.  Hint: Check you connection string.\r\n");
        return false;
    }
    // Setting message callback to get C2D messages
    (void)IoTHubDeviceClient_SetMessageCallback(iotHub->deviceHandle, receive_msg_callback, NULL);
    // Setting method callback to handle a SetTelemetryInterval method to control
    //   how often telemetry messages are sent from the simulated device.
    (void)IoTHubDeviceClient_SetDeviceMethodCallback(iotHub->deviceHandle, device_method_callback, NULL);
    // Setting connection status callback to get indication of connection to iothub
    (void)IoTHubDeviceClient_SetConnectionStatusCallback(iotHub->deviceHandle, connection_status_callback, NULL);
    (void)IoTHubDeviceClient_SetDeviceTwinCallback(iotHub->deviceHandle, deviceTwinCallback, iotHub);  //Device Twin callback requires context to send message and device Handle

    return true;
}

static void DeinitializeAzureIoTHub(ROS_Azure_IoT_Hub* iotHub)
{
    ROS_INFO("Deinitializing IoTHub Device client\r\n");
    IoTHubDeviceClient_Destroy(iotHub->deviceHandle);
    IoTHub_Deinit();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "universal_subscriber");

    ROS_Azure_IoT_Hub iotHub;

    if (!InitializeAzureIoTHub(&iotHub))
    {
        return -1;
    }

    ros::spin();

    DeinitializeAzureIoTHub(&iotHub);
    return 0;
}
