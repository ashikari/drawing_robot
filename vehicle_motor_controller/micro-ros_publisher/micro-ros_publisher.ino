#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>


// create publisher and subscriber objects
rcl_publisher_t publisher;
rcl_subscription_t subscriber;


geometry_msgs__msg__Twist cmdMsg;
geometry_msgs__msg__Twist stateMsg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscriber_callback(const void * msgin){
  const geometry_msgs__msg__Twist * cmdMsg = (const geometry_msgs__msg__Twist *)msgin;
    stateMsg.linear.x = cmdMsg->linear.x;
    stateMsg.linear.y = cmdMsg->linear.y;
    stateMsg.linear.z = cmdMsg->linear.z;
    stateMsg.angular.x = cmdMsg->angular.x;
    stateMsg.angular.y = cmdMsg->angular.y;
    stateMsg.angular.z = cmdMsg->angular.z;
    RCSOFTCHECK(rcl_publish(&publisher, &stateMsg, NULL));
}

// void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
// {  
//   RCLC_UNUSED(last_call_time);
//   if (timer != NULL) {
//     RCSOFTCHECK(rcl_publish(&publisher, &stateMsg, NULL));
//     stateMsg.linear.x += 10;
//     stateMsg.linear.y -= 10;
//     stateMsg.linear.z = 15;
//     stateMsg.angular.x = 0;
//     stateMsg.angular.y = 0;
//     stateMsg.angular.z +=1;
//   }
// }

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "onboard_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), // this macro is a bit weird
    "robot_state"));

   // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "robot_cmd_vel"));


  // // create timer,
  // const unsigned int timer_timeout = 1000;
  // RCCHECK(rclc_timer_init_default(
  //   &timer,
  //   &support,
  //   RCL_MS_TO_NS(timer_timeout),
  //   timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &cmdMsg, &subscriber_callback, ON_NEW_DATA));  
  // RCCHECK(rclc_executor_add_timer(&executor, &timer));


  // initialize values for published message
  stateMsg.linear.x = 0;
  stateMsg.linear.y = 0;
  stateMsg.linear.z = 0;
  stateMsg.angular.x = 0;
  stateMsg.angular.y = 0;
  stateMsg.angular.z = 0;
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
