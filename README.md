# Drone Base Firmware

> NOTE: This is a fun project meant for learning purposes and not intended for serious applications. However, if you are interested in building your own flight controller or drone from scratch, you may find some useful information here. Please note that the project was not successful.

## About

The Drone Base Firmware project is an open-source software project that aims to provide a complete flight controller solution for drone enthusiasts. The project is meant for learning purposes only and is not intended for commercial or industrial use.

This project is suitable for hobbyists, makers, and developers with a basic understanding of electronics and programming. It assumes some familiarity with microcontrollers, sensors, and communication protocols.

## Features

### Flight Controller

- Custom flight controller core.
- PID controller implementation.
- IMU support with sensor fusion using Madgwick's or Mahony's filters.
- BMP085 barometer for altitude measurement.
- Custom protocol based on Modbus-like protocol for long and short-range communication via RF and Bluetooth.

### Android/Windows App

[Android/Windows app](https://github.com/the-risk-taker/drone-short-range-control-app)

- Bluetooth app for on-field parameters changing, e.g., PID tuning.
- Made with Qt framework.
- Serial interface app for in-door testing.
- Made with Qt framework.

## Getting Started

The project was developed using [Atmel Studio 7](http://www.microchip.com/mplab/avr-support/atmel-studio-7) along with the [Visual Micro](https://www.visualmicro.com/) addon. However, it can also be run using the `Arduino IDE` by adding the necessary files to the project (although it may be necessary to modify the #include paths).

## Electronics components

- Arduino Due is the heart of drone,
- GY-80 IMU board:
  - ADXL345 accelerometer,
  - L3G4200D gyroscope,
  - HMC5883L magnetometer,
  - BMP085 barometer,
- Any ATmega 328P board (Uno and Pro Mini in my setup) for [remote control unit](https://github.com/the-risk-taker/drone-remote-firmware),
- nRF24L01 2.4GHz RF transceivers.

## Frame, motors, ESC and power supply (used by me)

- Tarot 650mm quadcopter frame TL65B01,
- Tarot low KV, 6S motors TL68P07,
- FVT LittleBee 30A ESC,
- 13" and 15" proppelers,
- 3S 5000mAh or 6S 5000mAh (I'm using 2x3S in series) for 13" and 15" propellers.

## Why wasn't the project successful?

I believe that the project's lack of success may be attributed to:

Low IMU quality; there are many better IMUs available (e.g., MPU9250).
Weak calibration of the sensors.
Poor implementation of the PID module.
Other factors.

## Acknowledgments

- Mahony and Madgwick filters libraries are made based on the [work](https://github.com/kriswiner/GY-80) of [Kris Winer](https://github.com/kriswiner).
