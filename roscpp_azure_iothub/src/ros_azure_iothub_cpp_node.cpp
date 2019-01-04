#include <stdlib.h>
#include <inttypes.h>
#include <stdio.h>

#include <thread>             // std::thread
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable

//Azure IoT Hub
#include "azure_c_shared_utility/macro_utils.h"
#include "azure_c_shared_utility/threadapi.h"
#include "azure_c_shared_utility/platform.h"
#include "iothub_device_client.h"
#include "iothub_client_options.h"
#include "iothub.h"
#include "iothub_message.h"
#include "parson.h"
#include "iothubtransportmqtt.h"

//ROS Introspection
#include "ros_type_introspection/ros_introspection.hpp"
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

using namespace RosIntrospection;

int g_interval = 10000;  // 10 sec send interval initially, currently not used
static size_t g_message_count_send_confirmations = 0;

struct ROS_Azure_IoT_Hub {
    IOTHUB_DEVICE_CLIENT_HANDLE deviceHandle;
    Parser parser;
    ros::NodeHandle nh;
    std::vector<ros::Subscriber> subscribers;
    std::mutex mtx;
    std::condition_variable cv;
	std::vector<std::string> topicsToSubscribe;
	int signal;
};

static IOTHUBMESSAGE_DISPOSITION_RESULT receive_msg_callback(IOTHUB_MESSAGE_HANDLE message, void* user_context)
{
    (void)user_context;
    const char* messageId;
    const char* correlationId;
	
	printf("receive_msg_callback\n");

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
            (void)printf("Failure retrieving byte array message\r\n");
        }
        else
        {
            (void)printf("Received Binary message\r\nMessage ID: %s\r\n Correlation ID: %s\r\n Data: <<<%.*s>>> & Size=%d\r\n", messageId, correlationId, (int)buff_len, buff_msg, (int)buff_len);
        }
    }
    else
    {
        const char* string_msg = IoTHubMessage_GetString(message);
        if (string_msg == NULL)
        {
            (void)printf("Failure retrieving byte array message\r\n");
        }
        else
        {
            (void)printf("Received String Message\r\nMessage ID: %s\r\n Correlation ID: %s\r\n Data: <<<%s>>>\r\n", messageId, correlationId, string_msg);
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

    (void)printf("\r\nDevice Method called for device %s\r\n", device_id);
    (void)printf("Device Method name:    %s\r\n", method_name);
    (void)printf("Device Method payload: %.*s\r\n", (int)size, (const char*)payload);

    if (strcmp(method_name, SetTelemetryIntervalMethod) == 0)
    {
        if (payload)
        {
            newInterval = (int)strtol((char*)payload, &end, 10);

            // Interval must be greater than zero.
            if (newInterval > 0)
            {
                // expect sec and covert to ms
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

    (void)printf("\r\nResponse status: %d\r\n", status);
    (void)printf("Response payload: %s\r\n\r\n", RESPONSE_STRING);

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
	printf("connection_status_callback\n");
    // This sample DOES NOT take into consideration network outages.
    if (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED)
    {
        (void)printf("The device client is connected to iothub\r\n");
    }
    else
    {
        (void)printf("The device client has been disconnected\r\n");
    }
}

static void send_confirm_callback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* userContextCallback)
{
    (void)userContextCallback;
    // When a message is sent this callback will get envoked
    g_message_count_send_confirmations++;
    (void)printf("Confirmation callback received for message %lu with result %s\r\n", (unsigned long)g_message_count_send_confirmations, ENUM_TO_STRING(IOTHUB_CLIENT_CONFIRMATION_RESULT, result));
}

static bool IsTopicAvailableForSubscribe(const char* topicName)
{
    printf("Trying to subscribe:  %s\n", topicName);
    
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
        printf("This topic has not been published yet: %s\n", topic_name.c_str() );
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
    (void)IoTHubMessage_SetProperty(message_handle, "property_key", "property_value");
    printf("\r\nSending message %d to IoTHub\r\nMessage: %s\r\n", (int)(messagecount + 1), msg);
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
    printf("topicCallback\n");
    const std::string&  datatype   =  msg->getDataType();
    const std::string&  definition =  msg->getMessageDefinition();

    // don't worry if you do this more than once: already registered message are not overwritten.
    parser.registerMessageDefinition( topic_name,
                                      RosIntrospection::ROSType(datatype),
                                      definition );

    // reuse these opbects to improve efficiency ("static" makes them persistent)
    static std::vector<uint8_t> buffer;
    static std::map<std::string,FlatMessage>   flat_containers;
    static std::map<std::string,RenamedValues> renamed_vectors;

    FlatMessage&   flat_container = flat_containers[topic_name];
    RenamedValues& renamed_values = renamed_vectors[topic_name];

    // copy raw memory into the buffer
    buffer.resize( msg->size() );
    ros::serialization::OStream stream(buffer.data(), buffer.size());
    msg->write(stream);

    // deserialize and rename the vectors
    parser.deserializeIntoFlatContainer( topic_name, absl::Span<uint8_t>(buffer), &flat_container, 100);
    parser.applyNameTransform( topic_name, flat_container, &renamed_values );

    // Print the content of the message
    printf("--------- %s ----------\n", topic_name.c_str() );
    sendMsgToAzureIoTHub(topic_name.c_str(), deviceHandle);
    for (auto it: renamed_values)
    {
        const std::string& key = it.first;
        const Variant& value   = it.second;
        char buffer [256] = {0};
        snprintf(buffer, sizeof(buffer), " %s = %f\n", key.c_str(), value.convert<double>());
        printf("%s",buffer);
        sendMsgToAzureIoTHub(buffer, deviceHandle);
    }
    for (auto it: flat_container.name)
    {
        const std::string& key    = it.first.toStdString();
        const std::string& value  = it.second;
        char buffer [256] = {0};
        snprintf(buffer, sizeof(buffer), " %s = %s\n", key.c_str(), value.c_str());
        printf("%s",buffer);
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

//Current code does not keep track of topics already subscribed
static void deviceTwinCallback(DEVICE_TWIN_UPDATE_STATE update_state, const unsigned char* payLoad, size_t size, void* userContextCallback)
{
    (void)update_state;
    (void)size;
	    
    ROS_Azure_IoT_Hub* iotHub = (ROS_Azure_IoT_Hub*)userContextCallback;

    JSON_Value* root_value = json_parse_string((const char*)payLoad);
    JSON_Object* root_object = json_value_get_object(root_value);
    JSON_Object* arrayObject = json_object_dotget_object (root_object, "desired.ros_relays");
    size_t objectCount = json_object_get_count(arrayObject);

    for (size_t i = 0; i < objectCount; i++)
    {
        printf("  %s \n", json_object_get_name(arrayObject, i));
        JSON_Value* value = json_object_get_value_at(arrayObject, i);
        const char* topicToSubscribe = json_value_get_string(value);   //  ?? Do we need to free this memory??
        printf("  %s \n", json_value_get_string(value));
        
        if (IsTopicAvailableForSubscribe(topicToSubscribe))
        {
            iotHub->topicsToSubscribe.push_back(topicToSubscribe);
        }
    }

    std::unique_lock<std::mutex> lck(iotHub->mtx);
    iotHub->signal = 1;
    iotHub->cv.notify_all();
}


static int InitializeAzureIoTHub(ROS_Azure_IoT_Hub* iotHub)
{
    IOTHUB_CLIENT_TRANSPORT_PROVIDER protocol = MQTT_Protocol;

    (void)IoTHub_Init();
    (void)printf("Creating IoTHub Device handle\r\n");

    ros::NodeHandle nh("~");
    std::string connectionString;
    nh.getParam("connection_string", connectionString);
    (void)printf("connection_string: %s\r\n", connectionString.c_str());

    // Create the iothub handle here
    iotHub->deviceHandle = IoTHubDeviceClient_CreateFromConnectionString(connectionString.c_str(), protocol);
    if (iotHub->deviceHandle == NULL)
    {
        (void)printf("Failure createing Iothub device.  Hint: Check you connection string.\r\n");
        return -1;
    }
    // Setting message callback to get C2D messages
    (void)IoTHubDeviceClient_SetMessageCallback(iotHub->deviceHandle, receive_msg_callback, NULL);
    // Setting method callback to handle a SetTelemetryInterval method to control
    //   how often telemetry messages are sent from the simulated device.
    (void)IoTHubDeviceClient_SetDeviceMethodCallback(iotHub->deviceHandle, device_method_callback, NULL);
    // Setting connection status callback to get indication of connection to iothub
    (void)IoTHubDeviceClient_SetConnectionStatusCallback(iotHub->deviceHandle, connection_status_callback, NULL);
    (void)IoTHubDeviceClient_SetDeviceTwinCallback(iotHub->deviceHandle, deviceTwinCallback, iotHub);  //Device Twin callback requires context to send message and device Handle

    return 0;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "universal_subscriber");
        
    ROS_Azure_IoT_Hub iotHub;
    iotHub.signal = 0;

    if (InitializeAzureIoTHub(&iotHub))
    {
        return -1;
    }

    //Wait for the first device twins callback and subscribe to the topics from this callback
    {
        std::unique_lock<std::mutex> lck(iotHub.mtx);
        iotHub.cv.wait(lck, [&iotHub]{return iotHub.signal != 0;});
    }

    for (auto& it: iotHub.topicsToSubscribe)
    {
       (void)printf("%s\r\n", it.c_str());
       subscribeTopic(it.c_str(), &iotHub);
    }

    ros::spin();
     
    IoTHubDeviceClient_Destroy(iotHub.deviceHandle);
    IoTHub_Deinit();

    return 0;
}
