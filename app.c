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
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h> // not used
// custom message:
#include <rigidbody_msgs/msg/robotcmd.h>


// macros functions, Constrain from arduino.h and RCCHECK from microros examples
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
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
#define stayOn 17
#define vSens ADC1_CHANNEL_5
#define ledb 4
#define ledg 16

// define motor pins
#define PinR1 21 // Forward right
#define PinR2 22 // Backwards right
#define PinL1 26 // Forward left
#define PinL2 25 // Backwards left

// Define the wheel radius and track width [m]
#define R 0.075
#define T 0.23

// PWM parameters
#define PWM_resolution LEDC_TIMER_10_BIT
#define PWM_timer LEDC_TIMER_1
#define PWM_FREQ 30000
#define PWM_spdMode LEDC_LOW_SPEED_MODE
#define PWM_min 600
#define PWM_max 1023

// setting PWM channels
#define PWM_R1 LEDC_CHANNEL_1
#define PWM_R2 LEDC_CHANNEL_2
#define PWM_L1 LEDC_CHANNEL_3
#define PWM_L2 LEDC_CHANNEL_4

#define timer_timeout 100

rcl_publisher_t publisher;
std_msgs__msg__Float32 battery_msg;

rcl_subscription_t subscriber;
rigidbody_msgs__msg__RobotCmd RobotCmd_msg;

void GPIOsetup();
void PWMsetup();
void motorControl(float vel, float a);
float batteryVoltage();
void cmd_vel_callback(const void *msgin);
void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void microRosTask();

void GPIOsetup()
{
    // Setup function for setting the GPIO settings
    // set internals
    gpio_set_direction(stayOn, GPIO_MODE_OUTPUT);
    gpio_set_direction(vSens, GPIO_MODE_INPUT);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(vSens, ADC_ATTEN_DB_0);
    gpio_set_level(stayOn, 1);

    // set all pwm outouts
    gpio_set_direction(PinL1, GPIO_MODE_OUTPUT);
    gpio_set_direction(PinL2, GPIO_MODE_OUTPUT);
    gpio_set_direction(PinR1, GPIO_MODE_OUTPUT);
    gpio_set_direction(PinR2, GPIO_MODE_OUTPUT);
}
void PWMsetup()
{
    // Setup function for setting the pwm channels, has to be called when
    // exiting sleepmode
    // pwm timer struct set to match motor drivers.
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = PWM_resolution,
        .freq_hz = PWM_FREQ,
        .speed_mode = PWM_spdMode,
        .timer_num = PWM_timer,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&ledc_timer);

    //filling the led_channel_config struct with the defined PWM parameters
    ledc_channel_config_t ledc_channel[4] = {
        {.channel = PWM_L1,
         .duty = 0,
         .gpio_num = PinL1,
         .speed_mode = PWM_spdMode,
         .hpoint = 0,
         .timer_sel = PWM_timer},

        {.channel = PWM_L2,
         .duty = 0,
         .gpio_num = PinL2,
         .speed_mode = PWM_spdMode,
         .hpoint = 0,
         .timer_sel = PWM_timer},

        {.channel = PWM_R1,
         .duty = 0,
         .gpio_num = PinR1,
         .speed_mode = PWM_spdMode,
         .hpoint = 0,
         .timer_sel = PWM_timer},

        {.channel = PWM_R2,
         .duty = 0,
         .gpio_num = PinR2,
         .speed_mode = PWM_spdMode,
         .hpoint = 0,
         .timer_sel = PWM_timer},
    };
    for (int i = 0; i < 4; i++)
    {
        ledc_channel_config(&ledc_channel[i]);
    }
}

void microRosTask()
{
    //Main task, running the microros system and handles subscriber information.

    rcl_allocator_t allocator = rcl_get_default_allocator();

    // microRos INIT options
    rclc_support_t support;
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "robot0", "", &support));

    // create subscriber, expecting msg in format of /cmd_vel, connect to topic /cmd_vel
    // initiated to connect with best effort for faster communication
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(rigidbody_msgs, msg, RobotCmd),
        "/robot0/cmd_vel"));
    // create publisher for returning batteryVoltage
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/batt_volt"));

    // Create timer.
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // Create Executor.
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &RobotCmd_msg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    battery_msg.data = 0;

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        usleep(10 * 1000);
    }
    // free mem
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

void cmd_vel_callback(const void *msgin)
{
    const rigidbody_msgs__msg__RobotCmd *RobotCmd_msg = (const rigidbody_msgs__msg__RobotCmd *)msgin;
    printf("Message received: %f %f\n", RobotCmd_msg->linear, RobotCmd_msg->angular);
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    if (timer == NULL)
    {
        // send battery data back
        battery_msg.data = batteryVoltage();
        RCSOFTCHECK(rcl_publish(&publisher, &battery_msg, NULL));
        printf("Sent: %f\n", battery_msg.data);
        return;
    }
    // run motor,
    else motorControl(RobotCmd_msg.linear, RobotCmd_msg.angular);
}
// Calculates the current battery voltage
float batteryVoltage()
{
    float vMeas = adc1_get_raw(ADC1_CHANNEL_5) * 3.3 / 4096;    // get voltage from adc
    float vBatt = vMeas * (100 * 10e3 + 22 * 10e3) / 22 * 10e3; // calculate battery voltage from voltagedivider circuit
    return vBatt;
}
void exitSleepMode(void)
{
    // exit sleepmode by writing a 100% dutycycle to each pin.
    ledc_set_duty(PWM_spdMode, PWM_L1, PWM_max);
    ledc_set_duty(PWM_spdMode, PWM_L2, PWM_max);
    ledc_set_duty(PWM_spdMode, PWM_R1, PWM_max);
    ledc_set_duty(PWM_spdMode, PWM_R2, PWM_max);
    //update the PWM channels
    ledc_update_duty(PWM_spdMode, PWM_L1);
    ledc_update_duty(PWM_spdMode, PWM_L2);
    ledc_update_duty(PWM_spdMode, PWM_R1);
    ledc_update_duty(PWM_spdMode, PWM_R2);
    // wait for 1ms for motor to enable
    vTaskDelay(1 / portTICK_RATE_MS);
}
void motorControl(float vel, float a)
{
    // Calculate the forward kinematics for the vehicle and write pwm

    // should we constrain both values to [-1,1]??
    // github.com/Reinbert/ros_esp32cam_diffdrive
    float norm_vel = constrain(vel, -1, 1);
    float norm_a = constrain(a, -1, 1);
    // Calculate wheel velocities
    float wL = (2.0f*vel - T*a)/(2.0f*R); // 2=> 2.0f?
    float wR = (2.0f*vel + T*a)/(2.0f*R);

    // map velocity to pwm
    uint16_t pwmLeft = (uint16_t)((fabs(wL)) * (PWM_max - PWM_min) / (1-0) + PWM_min);
    uint16_t pwmRight = (uint16_t)((fabs(wR)) * (PWM_max - PWM_min) / (1-0) + PWM_min);


    exitSleepMode();
    //sets the dutycycle of the motors to match the recieved wheel velocities.
    ledc_set_duty(PWM_spdMode, PWM_L1, pwmLeft * (wL > 0));
    ledc_set_duty(PWM_spdMode, PWM_L2, pwmLeft * (wL < 0));
    ledc_set_duty(PWM_spdMode, PWM_R1, pwmRight * (wR > 0));
    ledc_set_duty(PWM_spdMode, PWM_R2, pwmRight * (wR < 0));
    //update the PWM channels
    ledc_update_duty(PWM_spdMode, PWM_L1);
    ledc_update_duty(PWM_spdMode, PWM_L2);
    ledc_update_duty(PWM_spdMode, PWM_R1);
    ledc_update_duty(PWM_spdMode, PWM_R2);
}
void appMain()
{
    GPIOsetup();
    PWMsetup();
    microRosTask();
}