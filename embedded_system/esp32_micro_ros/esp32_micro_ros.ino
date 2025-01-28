#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_GIGA) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL) && !defined(ARDUINO_UNOR4_WIFI) && !defined(ARDUINO_OPTA)
#error This example is only available for Arduino Portenta, Arduino Giga R1, Arduino Nano RP2040 Connect, ESP32 Dev module, Wio Terminal, Arduino Uno R4 WiFi and Arduino OPTA WiFi 
#endif

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#if defined(LED_BUILTIN)
  #define LED_PIN LED_BUILTIN
#else
  #define LED_PIN 13
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define USE_WIFI true
bool init_comms();
bool init_serial_comms();
bool init_ros_comms(char* ssid, char* password, char* ip_address, uint32_t port, bool use_wifi);

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

bool init_comms()
{
  bool success = true;
  success &= init_serial_comms();
  success &= init_ros_comms("ssid", "password", "172.20.10.2", 8888, USE_WIFI);

  return success;
}

bool init_serial_comms()
{
  bool success = true;

  Serial.begin(115200);
  Serial.println("Serial communication started!");

  return success;
}

bool init_ros_comms(char* ssid, char* password, char* ip_address, uint32_t port, bool use_wifi = true)
{
  bool success = true;

  if(true == use_wifi)
  {
    Serial.print("Setting Wi-Fi Transports...");
    set_microros_wifi_transports(ssid, password, ip_address, port);
    Serial.print("done!\n");

    rmw_ret_t ping_response = rmw_uros_ping_agent(1000, 5);
    switch(ping_response)
    {
      case RMW_RET_OK:
        Serial.print("RMW_RET_OK...");
        break;
      case RMW_RET_ERROR:
        Serial.print("RMW_RET_ERROR...");
        break;
      case RMW_RET_TIMEOUT:
        Serial.print("RMW_RET_TIMEOUT...");
        break;
      case RMW_RET_UNSUPPORTED:
        Serial.print("RMW_RET_UNSUPPORTED...");
        break;
      case RMW_RET_BAD_ALLOC:
        Serial.print("RMW_RET_BAD_ALLOC...");
        break;
      case RMW_RET_INVALID_ARGUMENT:
        Serial.print("RMW_RET_INVALID_ARGUMENT...");
        break;
      case RMW_RET_INCORRECT_RMW_IMPLEMENTATION:
        Serial.print("RMW_RET_INCORRECT_RMW_IMPLEMENTATION...");
        break;
      case RMW_RET_NODE_NAME_NON_EXISTENT:
        Serial.print("RMW_RET_NODE_NAME_NON_EXISTENT...");
        break;
      default:
        break;
    }

    if (RMW_RET_OK == ping_response)
    {
      Serial.println("Connected to micro-ROS agent!");
      success = true;
    } 
    else 
    {
      Serial.println("Failed to connect to micro-ROS agent.");
      success = false;
    }
  }
  else
  {
    set_microros_transports();
  }

  return success;
}

void setup() {
  init_comms();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "agent_n", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "topic_name"));

  msg.data = 0;
}

void loop() {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
}

