// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
#pragma once

#include <frc/ADIS16448_IMU.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include <frc/Timer.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <rev/CANSparkMax.h>

#define PI 3.141592


class SensorModule {
    public:
        SensorModule(rev::SparkMaxRelativeEncoder* lEncoder, rev::SparkMaxRelativeEncoder* rEncoder); 
        double angle;
        double x;
        double y;


        nt::NetworkTableEntry distanceHC_SR04;



        int receiveInt();
        void update();

        //Sensors
        frc::ADIS16448_IMU* imu = new frc::ADIS16448_IMU();

        rev::SparkMaxRelativeEncoder* lEncoder; 
        rev::SparkMaxRelativeEncoder* rEncoder;
        





};
*/