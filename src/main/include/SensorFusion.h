// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#define PI 3.14159265

#include <vector>
#include <string>
#include <math.h> 
#include <wpi/numbers>

#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/ADIS16448_IMU.h>

class SensorFusion {
 public:
    //Constructor: Pass motors, gyro, and more...)
    SensorFusion(rev::SparkMaxRelativeEncoder* lEncoder, rev::SparkMaxRelativeEncoder* rEncoder);


    //Encoders to coord is really just encoders to theta(position code is not tested)
    void encoders_to_coord(double left, double right);

    //Sensor functions but they return between (0, 360)
    double get_magnometer();
    double get_gyro();

    //Sensor Drift Algorithm that works with module variables
    void gyro_drift();

    // Actual Functions used outside of class: 
    void initializeSensors();
    void updateSensors();
    double Get_trueGyro();


    //Sensor Drift
    double bound_num = 0.02; // Changeable: Bound for drift decrease
    double maginit;
    double max_drift = 0.4; // Changeable: Maximum drift value outside bound

    double mag_corrected; // This is really magnetic heading corrected
    double last_gyro_corrected = 0.0;

    double last_gyro = 0.0;
    double dbaseline = 26.0; //robot baseline(width between right and left wheels inches)


    //Position
    std::vector<double> position;


    //Gyro Stuff
    frc::ADIS16448_IMU* gyro_imu = new frc::ADIS16448_IMU;

    //Encoders
    rev::SparkMaxRelativeEncoder* lEncoder;
    rev::SparkMaxRelativeEncoder* rEncoder;

 private:
};

