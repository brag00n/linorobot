<img src="https://raw.githubusercontent.com/linorobot/lino_docs/master/imgs/wiki/logo1.png" width="200" height="200" />

# Bamboo linorobot [![Build Status](https://travis-ci.org/linorobot/lino_install.svg?branch=master)](https://travis-ci.org/linorobot/lino_install)
Linorobot is a suite of Open Source ROS compatible robots that aims to provide students, developers, and researchers a low-cost platform in creating new exciting applications on top of ROS.

"Bamboo Linorobot" is a fork from [linorobot/linorobot](https://github.com/linorobot/linorobot)

## Tutorial

You can read the full tutorial how to build your robot [here](https://github.com/grassjelly/linorobot/wiki/1.-Getting-Started).

## Bamboo Platform

types of robot base: 4WD
Works on Dockerized ROS Kinetic (Ubuntu 16.04)

## Hardware

#### Main ARM dev boards:    
- Raspberry Pi 3/B+ (Using Dockerized ROS system for managing sub-system Nodes and YDLIDAR X4)

#### Sub-system boards: 
 * ROS Serial Node01: Motion node (Board Teensy 4.0 managing Odometry HC-89 and Motor driver [Raspery MotorHat sku:418460](https://raspberrypiwiki.com/index.php/Robot_Expansion_Board))
 * ROS Serial Node02: Sensors node (Board GrovePi+ managing Ultrasound HC-SR40 and IMU MPU6050)
 * ROS Serial Node03: Camera node (Board M5StickV managing Camera and IMU)

## Firmware
Configurable sub-systems.
 * Node01 (Motion node) : linorobot_ws/firmware/rosserial01/lib/config/lino_base_config.h
 * Node02 (Sensor node) : linorobot_ws/firmware/rosserial02/lib/config/lino_base_config.h
 * Node03 (Camera node) : linorobot_ws/firmware/rosserial03/lib/config/lino_base_config.h
