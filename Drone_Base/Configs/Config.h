/*
* config.h
*
* Created: 03.06.2018 13:35:49
* Author: Mateusz Patyk
* Email: matpatyk@gmail.com
* Copyright © 2018 Mateusz Patyk. All rights reserved.
*/

#ifndef CONFIG_H_
#define CONFIG_H_

// TEST FILTERS
#define TEST_COMPLEMENTARY_VS_MAHONY_MADGWICK_FILTERS		0x00

// MOVING AVERAGE
#define ORIENTATION_MOVING_AVERAGE_BUFFER_SIZE				1
#define PITCH_OFFSET_VALUE									-1.75F
#define ROLL_OFFSET_VALUE									0.27F

// MIAN LOOP
#define DRONE_CONTROL_LOOP_FREQUENCY						200		// [Hz]

// LONG RANGE RADIO
#define NRF24L01_DRONE_REMOTE_RADIO_IDENTIFIER				0
#define NRF24L01_DRONE_BASE_RADIO_IDENTIFIER				1

// WATCHDOG:
#define DUE_WATCHDOG_TIMEOUT								1000 // [ms]

// COMM:
#define SHORT_RANGE_COMMUNICATION_SERIAL_BAUDRATE			115200UL
#define SHORT_RANGE_COMMUNICATION_BLUETOOTH_BAUDRATE		9600UL
#define SHORT_RANGE_COMMUNICATION_PROCESS_BYTES_INTERVAL	5

// ESC:
#define	FRONT_RIGHT_ESC_PWM_PIN								5	// 1
#define	FRONT_LEFT_ESC_PWM_PIN								4	// 4
#define	REAR_RIGHT_ESC_PWM_PIN								3	// 2
#define	REAR_LEFT_ESC_PWM_PIN								2	// 3

#define	FRONT_RIGHT_MOTOR_ID_LABEL							1#define	FRONT_LEFT_MOTOR_ID_LABEL							4#define	REAR_RIGHT_MOTOR_ID_LABEL							2#define	REAR_LEFT_MOTOR_ID_LABEL							3

#define	ESC_SAVE_PWM_VALUE									1000
#define	ESC_MIN_PWM_VALUE									1000
#define	ESC_MAX_PWM_VALUE									2000
#define ESC_TEST_SPEED										1000

#define THROTTLE_VALUE_WHEN_PID_IS_STARTING_TO_WORK			30

// PID
#define PID_INTEGRAL_STATE_VALUE							0.0F
#define PID_DERIVATE_STATE_VALUE							0.0F
#define PID_INTEGRAL_MIN_VALUE								-20.0F
#define PID_INTEGRAL_MAX_VALUE								20.0F
#define PID_PROPORTIONAL_GAIN_VALUE							2.0F
#define PID_INTEGRAL_GAIN_VALUE								0.000F
#define PID_DERIVATIVE_GAIN_VALUE							30.0F

// PINS:
#define NRF24L01_CE_PIN										9
#define NRF24L01_CSN_PIN									10
#define HMC5883L_A_DATA_READY_PIN							19
#define ADXL345_VCC_PIN										52


#endif /* CONFIG_H_ */