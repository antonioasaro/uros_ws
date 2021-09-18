#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/header.h>
#include <geometry_msgs/msg/twist.h>

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include "driver/gpio.h"

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#define SERVO_MIN_PULSEWIDTH 700 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2300 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate

#include "oled.h"

#define LED_GPIO 2
#define BUTTON_GPIO 35
#define SERVO_GPIO 18
#define STRING_BUFFER_LEN 50

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t button_publisher;
rcl_subscription_t cmd_vel_subscriber;

std_msgs__msg__Header outcoming_button;
geometry_msgs__msg__Twist cmd_vel;
uint32_t seq_no = 0;
uint32_t angle = 0;

static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

void button_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);

	if (timer != NULL) {

		gpio_set_level(LED_GPIO, seq_no++ & 0x1);
		sprintf(outcoming_button.frame_id.data, "antonio - %d", gpio_get_level(BUTTON_GPIO));
		outcoming_button.frame_id.size = strlen(outcoming_button.frame_id.data);

		// Fill the message timestamp
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		outcoming_button.stamp.sec = ts.tv_sec;
		outcoming_button.stamp.nanosec = ts.tv_nsec;

		// Reset the pong count and publish the button message
		rcl_publish(&button_publisher, (const void*)&outcoming_button, NULL);
		printf("Button send seq %s\n", outcoming_button.frame_id.data);

	}
}

void cmd_vel_subscription_callback(const void * msgin)
{
uint32_t pwm_us;

	geometry_msgs__msg__Twist * msg = (geometry_msgs__msg__Twist *) msgin;
    printf("Received linear: x==%f y==%f z==%f\n", msg->linear.x, msg->linear.y, msg->linear.z);
    printf("Received angular: x==%f y==%f z==%f\n", msg->angular.x, msg->angular.y, msg->angular.z);
    if (msg->linear.x > 0) { angle = angle + 1; }
    if (msg->linear.x < 0) { angle = angle - 1; }
	if ((angle >= 0) && (angle < SERVO_MAX_DEGREE)) {
		pwm_us = servo_per_degree_init(angle);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pwm_us);
        vTaskDelay(10);
 	}
 }


void appMain(void *argument)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "button_node", "", &support));

	// Create a reliable button publisher
	RCCHECK(rclc_publisher_init_default(&button_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/button"));

    	// Create a best effort cmd_vel subscriber
	RCCHECK(rclc_subscription_init_best_effort(&cmd_vel_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));


	// Create a 3 seconds button timer timer,
	rcl_timer_t timer;
	RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(2000), button_timer_callback));

	// Create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel, &cmd_vel_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	// Create and allocate the buttonpong messages
	char outcoming_button_buffer[STRING_BUFFER_LEN];
	outcoming_button.frame_id.data = outcoming_button_buffer;
	outcoming_button.frame_id.capacity = STRING_BUFFER_LEN;

	gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(BUTTON_GPIO);
    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);

	// Initializing mcpwm servo control gpio ...
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_GPIO);
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

	oled_main();

	while(1) {
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
		usleep(10000);
	}

	// Free resources
	RCCHECK(rcl_publisher_fini(&button_publisher, &node));
	RCCHECK(rcl_subscription_fini(&cmd_vel_subscriber, &node));

	RCCHECK(rcl_node_fini(&node));
}
