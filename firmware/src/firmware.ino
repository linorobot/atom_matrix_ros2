// Copyright (c) 2022 Juan Miguel Jimeno

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_utilities/string_utilities.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include "M5Atom.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

rcl_publisher_t imu_publisher;

sensor_msgs__msg__Imu imu_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t sensor_timer;

unsigned long long time_offset = 0;
bool micro_ros_init_successful = false;

void setup() 
{
    imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu_link");

    delay(1);
    M5.begin(false, true, true); 
    M5.IMU.Init(); 

    micro_ros_init_successful = false;
    
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
}

void loop() 
{
    static unsigned long prev_connect_test_time;
    // check if the agent got disconnected at 10Hz
    if(millis() - prev_connect_test_time >= 100)
    {
        prev_connect_test_time = millis();
        // check if the agent is connected
        if(RMW_RET_OK == rmw_uros_ping_agent(10, 2))
        {
            // reconnect if agent got disconnected or haven't at all
            if (!micro_ros_init_successful) 
            {
                createEntities();
            } 
        } 
        else if(micro_ros_init_successful)
        {
            // clean up micro-ROS components
            destroyEntities();
        }
    }
    
    if(micro_ros_init_successful)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
}

void sensorCallback(rcl_timer_t * timer, int64_t last_call_time) 
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {
       publishData();
    }
}

void createEntities()
{
    allocator = rcl_get_default_allocator();
    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, "imu_node", "", &support));
    
    // create IMU publisher
    RCCHECK(rclc_publisher_init_default( 
        &imu_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data_raw"
    ));
    
    // create timer for actuating the motors at 20 Hz (1000/50)
    const unsigned int sensor_timeout = 50;
    RCCHECK(rclc_timer_init_default( 
        &sensor_timer, 
        &support,
        RCL_MS_TO_NS(sensor_timeout),
        sensorCallback
    ));
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, & allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &sensor_timer));
    // synchronize time with the agent
    syncTime();
    micro_ros_init_successful = true;
}

void destroyEntities()
{
    rcl_publisher_fini(&imu_publisher, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&sensor_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    micro_ros_init_successful = false;
}

void publishData()
{
    float gx, gy, gz;
    float ax, ay, az;

    M5.IMU.getGyroData(&gx,&gy,&gz);
    M5.IMU.getAccelData(&ax,&ay,&az);

    struct timespec time_stamp = getTime();

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    imu_msg.angular_velocity.x = -gx * DEG_TO_RAD; 
    imu_msg.angular_velocity.y = -gy * DEG_TO_RAD; 
    imu_msg.angular_velocity.z = gz * DEG_TO_RAD; 

    imu_msg.angular_velocity_covariance[0] = 0.00001;
    imu_msg.angular_velocity_covariance[4] = 0.00001;
    imu_msg.angular_velocity_covariance[8] = 0.00001;
    
    imu_msg.linear_acceleration.x = ax; 
    imu_msg.linear_acceleration.y = ay; 
    imu_msg.linear_acceleration.z = az; 

    imu_msg.linear_acceleration_covariance[0] = 0.00001;
    imu_msg.linear_acceleration_covariance[4] = 0.00001;
    imu_msg.linear_acceleration_covariance[8] = 0.00001;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
}

void syncTime()
{
    // get the current time from the agent
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis(); 
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

void rclErrorLoop() 
{

}