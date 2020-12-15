<img src="https://raw.githubusercontent.com/linorobot/lino_docs/master/imgs/wiki/logo1.png" width="200" height="200" />

# Bamboo linorobot [![Build Status](https://travis-ci.org/linorobot/lino_install.svg?branch=master)](https://travis-ci.org/linorobot/lino_install)
Linorobot is a suite of Open Source ROS compatible robots that aims to provide students, developers, and researchers a low-cost platform in creating new exciting applications on top of ROS.

"Bamboo Linorobot" is a fork from [linorobot/linorobot](https://github.com/linorobot/linorobot)

## Tutorial

You can read the full tutorial how to build your robot [here](https://github.com/grassjelly/linorobot/wiki/1.-Getting-Started).

## Bamboo Platform
types of robot base: 4WD

![alt text](https://github.com/linorobot/lino_docs/blob/master/imgs/readme/family.png?raw=true)

Works on:
- ROS Kinetic (Ubuntu 16.04)

## Hardware

#### Main ARM dev boards:    
- Raspberry Pi 3/B+ (Using ROS sustem, Manage sub-system Nodes and YDLIDAR X4)

#### Sub-system boards: 
 * Node01: Teensy 4.0 (Motion node: Odometry HC-89, Motor driver [Raspery MotorHat sku:418460](https://raspberrypiwiki.com/index.php/Robot_Expansion_Board))
 * Node02: GrovePi+   (Sensors node: Ultrasound HC-SR40; IMU MPU6050)
 * Node03: M5StickV   (Camera node: Camera; IMU)

## Firmware
Configurable subsystems.
 * Node01 (Motion node) : linorobot_ws/firmware/node01/lib/config/lino_base_config.h
 * Node02 (Sensor node) : linorobot_ws/firmware/node02/lib/config/lino_base_config.h
 * Node03 (Camera node) : linorobot_ws/firmware/node03/lib/config/lino_base_config.h
