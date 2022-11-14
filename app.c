// standart C libs
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <inttypes.h>
#include <math.h>

// esp gpio
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <driver/adc.h>
// free Rtos
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// esp sys
#include "esp_system.h"

// microros util
#include <ros.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>

#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK))                                                     \
        {                                                                                \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            vTaskDelete(NULL);                                                           \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK))                                                       \
        {                                                                                  \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }

// define system pins
#define stayOn GPIO_NUM_17
#define vSens ADC1_CHANNEL_5
#define ledb GPIO_NUM_4
#define ledg GPIO_NUM_16

// define motor pins
#define PinR1 GPIO_NUM_21
#define PinR2 GPIO_NUM_22
#define PinL1 GPIO_NUM_26
#define PinL2 GPIO_NUM_25

// PWM parameters
#define PWM_resolution LEDC_TIMER_10_BIT
#define PWM_timer LEDC_TIMER_1
#define PWM_spdMode LEDC_LOW_SPEED_MODE

// setting PWM channels
#define PWM_R1 LEDC_CHANNEL_0
#define PWM_R2 LEDC_CHANNEL_1
#define PWM_L1 LEDC_CHANNEL_2
#define PWM_L2 LEDC_CHANNEL_3

static rcl_publisher_t publisher;
std_msgs__msg__Float32 battery_msg;

static rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;



void initMsg();
float batteryVoltage();
void cmd_vel_callback();
void timer_callback();
void setup();
void microRosTask();

void setup()
{
    // TO DO define struct for setting motors
    gpio_set_direction(stayOn, GPIO_MODE_OUTPUT);
    gpio_set_direction(vSens, GPIO_MODE_INPUT);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(vSens, ADC_ATTEN_DB_0);
    gpio_set_level(stayOn, 1);
}
void microRosTask()
{
    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    rcl_allocator_t allocator = rcl_get_default_allocator();

    // microRos INIT options
    rclc_support_t support;
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "LanciersBot", "", &support));

    // create subscriber, expecting msg in format of /cmd_vel, connect to topic /cmd_vel
    RCCHECK(rclc_subscription_init_default(
        // TO DO, create publisher for returning battery voltage
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));
    // create publisher for returning batteryVoltage

    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/batt_volt"));

    // Create timer.
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // Create Executor.
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    battery_msg.data = 0;

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1000));
        usleep(RCL_MS_TO_NS(1000));
    }
    // free mem
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

void cmd_vel_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    printf("Message received: %f %f\n", msg->linear.x, msg->angular.z);
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    // TO DO:
    // Write program for controlling motors
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        battery_msg.data = batteryVoltage();
        RCSOFTCHECK(rcl_publish(&publisher, &battery_msg, NULL));
        printf("Sent: %f\n", battery_msg.data);
    }
}
// Calculates the current battery voltage
float batteryVoltage()
{
    float vMeas = adc1_get_raw(ADC1_CHANNEL_5) * 3.3 / 4096;    // get voltage from adc
    float vBatt = vMeas * (100 * 10e3 + 22 * 10e3) / 22 * 10e3; // calculate battery voltage from voltagedivider circuit
    return vBatt;
}

void appMain()
{
    setup();
    initMsg();
    microRosTask();
}
